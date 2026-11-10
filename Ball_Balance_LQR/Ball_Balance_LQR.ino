/*               ****NOTES****
  Maestro Control Center -> Serial Settings:
   Serial mode: UART, fixed baud rate
   Baud rate: 115200 (must match maestroSerial.begin below)
   CRC disabled
  Click "Apply Settings" after changes.

  Servo array indexes: a1=0 a2=1 b1=2 b2=3 c1=4 c2=5

  LQR CONTROLLER + PATTERN MODES
  - The ball is controlled by an LQR state feedback (position + velocity error
    -> tilt angle). The error is always  ball - SET POINT, so the set point can
    be a fixed point, a circle or a star; the controller just tracks it.
  - x and y are independent and identical (same ball dynamics), so one small
    2-state LQR is solved per axis (no matrix library needed). The gains are
    computed once at start-up (and again after the "l" command).
  - Serial commands (Serial Monitor, set PLOT = false to see replies):
      "o"          start / stop the circle with the current settings
      "o 40 10"    circle radius 40 px, 10 s per revolution, and start
      "o 40 -10"   negative time = other direction
      "c"          stop any pattern and go back to the centre
      "p"          star (pentagram) on / off
      "p 50 25"    star: corner radius 50 px, set point speed 25 px/s
      "l"          print the LQR gains
      "l 1 1 100"  new LQR weights: q_pos q_vel r  (then gains are recomputed)
      "s x y"      fixed point (stops the circle), e.g. "s 30 -20"
      "30 -20"     same, the "s" is optional
                   (x/y in px from the platform centre, +x right, +y up,
                    limited to sp_max)
  - The radius ramps up from 0 (spiral out), so the ball is not thrown around.
  - Start slow (T = 10..15 s) and small (R = 30..40 px), then increase.
*/

#include "PololuMaestro.h"
#include "math.h"
#include <Pixy2.h>

#define maestroSerial SERIAL_PORT_HARDWARE_OPEN  // for teensy

// set to true for verbose serial output (slows the loop down)
const bool DEBUG = false;

// true = print ONLY "label:value" data for the Serial Plotter (all text
// messages are suppressed, otherwise they break the plot).
// false = normal text messages for the Serial Monitor.
const bool PLOT = true;

// ------------------- START TUNE -------------------

// ANGLE OFFSET FOR EACH MOTOR [NEGATIVE UP / POSITIVE DOWN] - DEGREES
float offset[6] = { -7, 0.0,
                    -2, -1,
                    0.0, 0.0 };

// CAMERA CALIBRATION
float origin[2] = { 157, 103 };
float r_platform = 124;

// SET POINT: where the ball should be held, in pixels RELATIVE to the platform
// centre (origin). +x = right, +y = up (same axes as the error).
float setpoint[2] = { 0, 0 };
float sp_max = 90;  // limit for the setpoint radius (px), keep it inside r_platform

// CIRCLE: the set point travels on a circle around the platform centre
float circle_R = 40;     // radius in px (max sp_max, keep well inside r_platform)
float circle_T = 10;     // seconds per revolution (negative = reverse direction)
float circle_ramp = 3;   // seconds to grow from radius 0 to circle_R
// true = circle starts by itself at power-up (no serial command needed, so it
// also works with the Serial Plotter, which cannot send anything)
// Set it to false if you want to start in point mode (set point = centre) and
// type a point or "o" in the Serial Monitor yourself.
const bool CIRCLE_ON_BOOT = false;

// STAR (pentagram): the set point travels along straight lines through 5 corners
float star_R = 50;       // radius of the corners in px (max sp_max)
float star_speed = 25;   // speed of the set point in px/s

// HOME pose. hz_home MUST be reachable: ~119..180 mm with p1=31, p2=150.
float hx_home = 0;
float hy_home = 0;
float hz_home = 145;
float roll_home = 0;
float pitch_home = 0;
float yaw_home = 0;

// MAX AVERAGE ANGLE (pitch/roll) IN ONE STEP - DEGREES
float r_max = 15;

// LQR  (per axis: state = [position error (px), velocity error (px/s)],
//       input = tilt angle in DEGREES, cost = sum(q_pos*e^2 + q_vel*v^2 + r*u^2))
// Larger q_* = tighter tracking, larger r = gentler / slower tilting.
// These defaults give gains close to the old PID (about 0.085 deg/px, 0.09 deg/(px/s)).
float q_pos = 1.0;       // penalty on position error
float q_vel = 1.0;       // penalty on velocity error
float r_lqr = 100.0;     // penalty on tilt angle (deg^2)
float dt_nom = 0.02;     // nominal Pixy frame time (s) used for the gain design

// plant model used by the LQR:  ball acceleration = -c * tilt
float g_mm = 9800.0;               // gravity (mm/s^2)
float px_per_mm = 124.0 / 100.0;   // camera pixels per mm (r_platform px / 100 mm)
float ball_k = 5.0 / 7.0;          // 5/7 = solid ball, 2/3 = hollow ball

// optional integral action (0 = off). Removes a constant offset, e.g. when the
// platform is not perfectly level. Use small values (0.001 .. 0.01).
float ki = 0.0;
float integral_max = 200;  // anti-windup clamp (pixel*s)

// Low pass filter constant (s)
float T = 0.08;

// MOTOR VELOCITIES AND ACCELERATIONS
int vel_home = 20;
int acc_home = 20;
int vel_control = 0;
int acc_control = 0;

// MEASUREMENTS (mm)
float l0 = 55;
float d1 = 55;
float lf = 65;
float m = 12.7;
float p1 = 31;   // motor small arm
float p2 = 150;  // motor long arm

// ------------------- END TUNE -------------------

Pixy2 pixy;
MicroMaestro maestro(maestroSerial);

// **CONSTANTS**
float abs_0 = 4000;
float abs_90 = 8000;
float toDeg = 180 / PI;
float toRad = 1 / toDeg;
int x = 0, y = 1, z = 2;
String ID[6] = { "a1", "a2", "b1", "b2", "c1", "c2" };

float range[6][2] = { { -45, 45 }, { 45, -45 },
                      { -45, 45 }, { 45, -45 },
                      { -45, 45 }, { 45, -45 } };

// **INVERSE KINEMATICS CONSTANTS**
float a10[3] = { -d1, -l0, 0 };
float a20[3] = { -a10[x], a10[y], 0 };
float b10[3] = { l0 * sqrt(3.0f) / 2.0f + d1 / 2.0f, l0 / 2.0f - d1 * sqrt(3.0f) / 2.0f, 0 };
float b20[3] = { l0 * sqrt(3.0f) / 2.0f - d1 / 2.0f, l0 / 2.0f + d1 * sqrt(3.0f) / 2.0f, 0 };
float c10[3] = { -b20[x], b20[y], 0 };
float c20[3] = { -b10[x], b10[y], 0 };

float a_10_20[3] = { a20[x] - a10[x], a20[y] - a10[y], a20[z] - a10[z] };
float b_10_20[3] = { b20[x] - b10[x], b20[y] - b10[y], b20[z] - b10[z] };
float c_10_20[3] = { c20[x] - c10[x], c20[y] - c10[y], c20[z] - c10[z] };

float na[3] = { 0, -1, 0 };
float nb[3] = { sqrt(3.0f) / 2.0f, 1 / 2.0f, 0 };
float nc[3] = { -sqrt(3.0f) / 2.0f, 1 / 2.0f, 0 };

float t = lf / 2.0f + m * sqrt(3.0f) / 2.0f;
float w = lf * sqrt(3.0f) / 2.0f - m / 2.0f;

float theta[6] = { 0, 0, 0, 0, 0, 0 };  // last valid servo angles

// **PIXY2**
float ball[2];
bool ballFound = false;

// **PID**
float error[2] = { 0, 0 };
float error_prev[2] = { 0, 0 };
float deriv[2] = { 0, 0 };
float integral[2] = { 0, 0 };
float out[2];
unsigned long time_i = 0;
bool tracking = false;  // true while PID state is valid
bool atHome = false;    // true once the home pose has been sent

// **CIRCLE STATE**
bool circleOn = false;
float circleAngle = 0;       // rad
float circleR_now = 0;       // current (ramped) radius in px
unsigned long circleLast = 0;

// **STAR STATE**
bool starOn = false;
int starIdx = 0;             // index of the corner we are heading to
unsigned long starLast = 0;

// **LQR GAINS**  tilt[deg] = lqr_K[0]*error[px] + lqr_K[1]*error_velocity[px/s]
float lqr_K[2] = { 0, 0 };

// forward declarations
bool InverseKinematics(float hx, float hy, float hz, float roll, float pitch, float yaw);

float fmap(float v, float in_min, float in_max, float out_min, float out_max) {
  return (v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);  // short timeout for parseFloat() in handleSerial()
  maestroSerial.begin(115200);
  pixy.init();
  computeLQR();
  if (!PLOT) printLQR();

  // sanity check of the home pose
  if (InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home)) {
    moveServos(vel_home, acc_home);
    atHome = true;
  } else {
    if (!PLOT) Serial.println("WARNING: home pose not reachable! Change hz_home (approx. 119..180 mm).");
  }
  if (CIRCLE_ON_BOOT) startCircle();
}

// Change the set point (pixels relative to platform centre).
// Shifts the stored previous error by the same amount so a set point step does
// not cause a derivative kick, and clears the integral.
void setSetpoint(float sx, float sy) {
  float r = sqrt(sx * sx + sy * sy);
  if (r > sp_max) {  // keep it on the platform
    sx *= sp_max / r;
    sy *= sp_max / r;
  }
  error_prev[x] -= (sx - setpoint[x]);
  error_prev[y] -= (sy - setpoint[y]);
  setpoint[x] = sx;
  setpoint[y] = sy;
  integral[x] = integral[y] = 0;
  if (!PLOT) {
    Serial.print("Setpoint: ");
    Serial.print(setpoint[x]);
    Serial.print(", ");
    Serial.println(setpoint[y]);
  }
}

// ---------------- LQR DESIGN ----------------
// One axis, discrete time with step dt:
//   x = [pos, vel],  x+ = A x + B u,   A = [[1, dt], [0, 1]],  B = [-c*dt^2/2, -c*dt]
// c = ball acceleration per degree of tilt (px/s^2/deg). The minus sign matches
// the sign convention of the working PID (positive error -> positive tilt).
// The Riccati equation is iterated until it converges; K = -(R+B'PB)^-1 B'PA.
void computeLQR() {
  float c = g_mm * px_per_mm * ball_k * PI / 180.0f;
  float dt = dt_nom;
  float b0 = -c * dt * dt / 2.0f;
  float b1 = -c * dt;
  float p00 = q_pos, p01 = 0, p11 = q_vel;
  float v0 = 0, v1 = 0, S = 1;

  for (int i = 0; i < 3000; i++) {
    float w0 = p00 * b0 + p01 * b1;            // P*B
    float w1 = p01 * b0 + p11 * b1;
    S = r_lqr + b0 * w0 + b1 * w1;             // R + B'PB (scalar)
    v0 = w0;                                   // A'*P*B
    v1 = dt * w0 + w1;
    float a00 = p00;                           // A'*P*A
    float a01 = dt * p00 + p01;
    float a11 = dt * dt * p00 + 2.0f * dt * p01 + p11;
    float n00 = a00 + q_pos - v0 * v0 / S;
    float n01 = a01 - v0 * v1 / S;
    float n11 = a11 + q_vel - v1 * v1 / S;
    p00 = n00; p01 = n01; p11 = n11;
  }
  // gains from the converged P
  float w0 = p00 * b0 + p01 * b1;
  float w1 = p01 * b0 + p11 * b1;
  S = r_lqr + b0 * w0 + b1 * w1;
  v0 = w0;
  v1 = dt * w0 + w1;
  lqr_K[0] = -v0 / S;
  lqr_K[1] = -v1 / S;
}

void printLQR() {
  Serial.print("LQR gains: Kpos=");
  Serial.print(lqr_K[0], 5);
  Serial.print(" deg/px, Kvel=");
  Serial.print(lqr_K[1], 5);
  Serial.println(" deg/(px/s)");
}

// ---------------- PATTERNS ----------------
void startCircle() {
  starOn = false;  // only one pattern at a time
  circleOn = true;
  circleAngle = 0;
  circleR_now = 0;  // spiral out from the centre
  circleLast = millis();
  if (!PLOT) Serial.println("Circle ON");
}

// stops circle AND star (also used by the point commands)
void stopCircle() {
  starOn = false;
  if (!circleOn) return;
  circleOn = false;
  if (!PLOT) Serial.println("Circle OFF");
}

// corner k of the pentagram, visited in the order 0,2,4,1,3 (the star shape)
void starVertex(int k, float &vx, float &vy) {
  static const int order[5] = { 0, 2, 4, 1, 3 };
  float a = (90.0f + 72.0f * order[k % 5]) * toRad;  // first corner at the top
  vx = star_R * cos(a);
  vy = star_R * sin(a);
}

void startStar() {
  circleOn = false;  // only one pattern at a time
  starOn = true;
  starIdx = 0;       // first move: from the current set point to the top corner
  starLast = millis();
  if (!PLOT) Serial.println("Star ON");
}

// Moves the set point towards the next corner. Called at the start of every
// control cycle, before the error is computed.
void updateStar() {
  if (!starOn) return;
  unsigned long now = millis();
  float dtc = (now - starLast) / 1000.0f;
  starLast = now;
  if (dtc > 0.1f) dtc = 0.1f;  // e.g. after the ball was lost: no jump

  float step = star_speed * dtc;
  float tx, ty;
  starVertex(starIdx, tx, ty);
  float dx = tx - setpoint[x];
  float dy = ty - setpoint[y];
  float dist = sqrt(dx * dx + dy * dy);
  if (dist <= step) {  // corner reached -> next corner
    setpoint[x] = tx;
    setpoint[y] = ty;
    starIdx = (starIdx + 1) % 5;
  } else {
    setpoint[x] += dx / dist * step;
    setpoint[y] += dy / dist * step;
  }
}

// Moves the set point along the circle. Called at the start of every PID cycle,
// BEFORE the error is computed. The set point is written directly (no
// setSetpoint()): the error derivative then automatically becomes
// (ball velocity - set point velocity), which is exactly what we want when
// tracking a moving target.
void updateCircle() {
  if (!circleOn) return;
  unsigned long now = millis();
  float dtc = (now - circleLast) / 1000.0f;
  circleLast = now;
  if (dtc > 0.1f) dtc = 0.1f;  // e.g. after the ball was lost: no jump

  // ramp the radius up (and follow later changes of circle_R)
  float R_target = constrain(circle_R, 0, sp_max);
  if (circleR_now < R_target) {
    circleR_now += R_target / max(circle_ramp, 0.1f) * dtc;
    if (circleR_now > R_target) circleR_now = R_target;
  } else {
    circleR_now = R_target;
  }

  // advance the angle (negative circle_T = reverse direction)
  if (fabs(circle_T) < 0.5f) circle_T = (circle_T < 0) ? -0.5f : 0.5f;  // safety
  circleAngle += 2.0f * PI / circle_T * dtc;
  if (circleAngle > 2.0f * PI) circleAngle -= 2.0f * PI;
  if (circleAngle < -2.0f * PI) circleAngle += 2.0f * PI;

  setpoint[x] = circleR_now * cos(circleAngle);
  setpoint[y] = circleR_now * sin(circleAngle);
}

// Serial commands:
//   "30 -20" or "s 30 -20"  fixed point        "c"  centre (stops circle/star)
//   "o"          circle on/off      "o <R> <T>"  circle with radius R px, T s
//   "p"          star on/off        "p <R> <V>"  star with radius R px, speed V px/s
//   "l"          print LQR gains    "l <qp> <qv> <r>"  new LQR weights
void handleSerial() {
  if (!Serial.available()) return;
  char cmd = Serial.peek();
  if (cmd == '\n' || cmd == '\r' || cmd == ' ') {  // ignore line endings / spaces
    Serial.read();
    return;
  }
  // a line starting with a number is a plain point: "30 -20"
  bool plain = (cmd == '-' || cmd == '+' || cmd == '.' || (cmd >= '0' && cmd <= '9'));
  if (!plain) Serial.read();  // consume the command letter

  if (plain || cmd == 's') {
    float sx = Serial.parseFloat();
    float sy = Serial.parseFloat();
    stopCircle();
    setSetpoint(sx, sy);
  } else if (cmd == 'c') {
    stopCircle();
    setSetpoint(0, 0);
  } else if (cmd == 'o') {
    float R = Serial.parseFloat();  // 0 if nothing was sent
    float Tn = Serial.parseFloat();
    if (R > 0 && Tn != 0) {
      circle_R = constrain(R, 5, sp_max);
      circle_T = Tn;
      startCircle();
    } else if (circleOn) {
      stopCircle();
      setSetpoint(0, 0);
    } else {
      startCircle();
    }
  } else if (cmd == 'p') {
    float R = Serial.parseFloat();
    float V = Serial.parseFloat();
    if (R > 0 && V > 0) {
      star_R = constrain(R, 10, sp_max);
      star_speed = constrain(V, 5, 200);
      startStar();
    } else if (starOn) {
      stopCircle();  // also stops the star
      setSetpoint(0, 0);
    } else {
      startStar();
    }
  } else if (cmd == 'l') {
    float qp = Serial.parseFloat();
    float qv = Serial.parseFloat();
    float rr = Serial.parseFloat();
    if (qp > 0 && qv >= 0 && rr > 0) {
      q_pos = qp;
      q_vel = qv;
      r_lqr = rr;
      computeLQR();
    }
    printLQR();  // printed even in PLOT mode (you asked for it)
  }
  while (Serial.available()) Serial.read();  // drop the rest of the line
}

void loop() {
  handleSerial();
  findBall();
  if (!ballFound) {
    tracking = false;  // reset PID memory
    if (!atHome) goHome();
  } else {
    LQR();
  }
}

void goHome() {
  if (InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home)) {
    moveServos(vel_home, acc_home);
    atHome = true;
  }
}

// move servo i to an input position (degrees)
void moveServo(int i, float pos, int spd, int acc) {
  pos = pos + offset[i];
  pos = fmap(pos, range[i][0], range[i][1], abs_0, abs_90);
  pos = constrain(pos, 3600, 8400);
  maestro.setSpeed(i, spd);
  maestro.setAcceleration(i, acc);
  maestro.setTarget(i, (uint16_t)pos);
}

// move all servos to theta[]
void moveServos(int spd, int acc) {
  for (int i = 0; i < 6; i++) {
    moveServo(i, theta[i], spd, acc);
  }
}

// find the ball; if several blocks are seen, use the largest one
void findBall() {
  pixy.ccc.getBlocks();
  int n = pixy.ccc.numBlocks;
  if (n <= 0) {
    ballFound = false;
    return;
  }
  int best = 0;
  long bestArea = 0;
  for (int i = 0; i < n; i++) {
    long area = (long)pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
    if (area > bestArea) {
      bestArea = area;
      best = i;
    }
  }
  if (DEBUG && n > 1) Serial.println("Multiple blocks, using largest");
  ball[x] = pixy.ccc.blocks[best].m_x;
  ball[y] = pixy.ccc.blocks[best].m_y;
  ballFound = true;
}

void LQR() {
  updateCircle();  // move the set point along the circle (if active)
  updateStar();    // move the set point along the star (if active)

  // ball position relative to the platform centre (y flipped: camera y is down)
  float pos_x = ball[x] - origin[x];
  float pos_y = -(ball[y] - origin[y]);

  // error = ball position - set point
  error[x] = pos_x - setpoint[x];
  error[y] = pos_y - setpoint[y];

  // ball outside the platform (measured from the centre, not the set point) -> home
  float r_ball = sqrt(pos_x * pos_x + pos_y * pos_y);
  if (r_ball > r_platform) {
    if (DEBUG) Serial.println("Ball out of platform");
    tracking = false;
    if (!atHome) goHome();
    return;
  }

  unsigned long now = millis();

  // (re)start tracking: initialise state, skip derivative for this frame
  if (!tracking) {
    error_prev[x] = error[x];
    error_prev[y] = error[y];
    deriv[x] = deriv[y] = 0;
    integral[x] = integral[y] = 0;
    time_i = now;
    tracking = true;
    return;
  }

  float dt = (now - time_i) / 1000.0f;
  if (dt < 0.001f) return;  // too fast, wait for the next frame
  time_i = now;

  for (int k = 0; k < 2; k++) {
    float raw = (error[k] - error_prev[k]) / dt;
    deriv[k] += dt / T * (-deriv[k] + raw);  // low-pass filtered derivative
    if (isnan(deriv[k]) || isinf(deriv[k])) deriv[k] = 0;

    integral[k] += (error[k] + error_prev[k]) * dt / 2.0f;  // trapezoidal
    integral[k] = constrain(integral[k], -integral_max, integral_max);

    error_prev[k] = error[k];
    // LQR state feedback: tilt[deg] = K * [error, error velocity]
    out[k] = lqr_K[0] * error[k] + lqr_K[1] * deriv[k] + ki * integral[k];
  }

  // limit the tilt
  float r_out = sqrt(out[x] * out[x] + out[y] * out[y]);
  if (r_out > r_max) {
    out[x] *= r_max / r_out;
    out[y] *= r_max / r_out;
  }

  // Serial Plotter output: one line per sample, "label:value" pairs
  // (plot ball_x/set_x or ball_y/set_y to see the circle as two sine waves)
  if (PLOT) {
    Serial.print("ball_x:");   Serial.print(pos_x);        Serial.print(',');
    Serial.print("ball_y:");   Serial.print(pos_y);        Serial.print(',');
    Serial.print("set_x:");    Serial.print(setpoint[x]);  Serial.print(',');
    Serial.print("set_y:");    Serial.print(setpoint[y]);  Serial.print(',');
    Serial.print("err_x:");    Serial.print(error[x]);     Serial.print(',');
    Serial.print("err_y:");    Serial.print(error[y]);     Serial.print(',');
    Serial.print("pitch:");    Serial.print(out[x]);       Serial.print(',');
    Serial.print("roll:");     Serial.println(-out[y]);
  }

  // out[x] -> pitch(+), out[y] -> roll(-)
  if (InverseKinematics(hx_home, hy_home, hz_home, roll_home - out[y], pitch_home + out[x], yaw_home)) {
    moveServos(vel_control, acc_control);
    atHome = false;
  }
}

// Solves one servo. v = vector from the servo pivot to the platform joint,
// n = outward normal of that side, axis = a_10_20 etc., sgn = +1 / -1.
// Returns false if the pose is not reachable (no NaN is ever produced).
bool solveLeg(const float v[3], const float n[3], const float axis[3], float sgn, float &thetaDeg) {
  float s_mod = fabs(dot(v, n));
  float proj[3] = { v[0] + n[0] * s_mod, v[1] + n[1] * s_mod, v[2] + n[2] * s_mod };
  float proj_mod = mag(proj);
  float p2sq = p2 * p2 - s_mod * s_mod;
  if (p2sq <= 0 || proj_mod < 1e-3f) return false;
  float p2_proj_mod = sqrt(p2sq);

  float c1 = sgn * dot(proj, axis) / (2 * d1 * proj_mod);
  float c2 = (proj_mod * proj_mod + p1 * p1 - p2_proj_mod * p2_proj_mod) / (2 * proj_mod * p1);

  // outside [-1,1] = arm cannot reach the point
  if (fabs(c1) > 1.0001f || fabs(c2) > 1.0001f) return false;
  c1 = constrain(c1, -1.0f, 1.0f);
  c2 = constrain(c2, -1.0f, 1.0f);

  thetaDeg = (acos(c1) - acos(c2)) * toDeg;
  return true;
}

// Inverse kinematics. On success theta[] is updated and true is returned.
// On failure theta[] is left untouched (platform keeps its last good pose).
bool InverseKinematics(float hx, float hy, float hz, float roll, float pitch, float yaw) {
  float h[3] = { hx, hy, hz };

  float R_z_yaw[3][3] = { { cos(yaw * toRad), -sin(yaw * toRad), 0 }, { sin(yaw * toRad), cos(yaw * toRad), 0 }, { 0, 0, 1 } };
  float R_y_pitch[3][3] = { { cos(pitch * toRad), 0, sin(pitch * toRad) }, { 0, 1, 0 }, { -sin(pitch * toRad), 0, cos(pitch * toRad) } };
  float R_x_roll[3][3] = { { 1, 0, 0 }, { 0, cos(roll * toRad), -sin(roll * toRad) }, { 0, sin(roll * toRad), cos(roll * toRad) } };

  float ha_i[3] = { 0, -t, 0 };
  float hb_i[3] = { sqrt(3.0f) * t / 2.0f, t / 2.0f, 0 };
  float hc_i[3] = { -sqrt(3.0f) * t / 2.0f, t / 2.0f, 0 };
  float ha[3], hb[3], hc[3];
  rotations(ha, ha_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(hb, hb_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(hc, hc_i, R_z_yaw, R_y_pitch, R_x_roll);

  float af[3] = { h[0] + ha[0], h[1] + ha[1], h[2] + ha[2] };
  float bf[3] = { h[0] + hb[0], h[1] + hb[1], h[2] + hb[2] };
  float cf[3] = { h[0] + hc[0], h[1] + hc[1], h[2] + hc[2] };

  float af_a1f_i[3] = { -w, 0, 0 };
  float af_a2f_i[3] = { w, 0, 0 };
  float bf_b1f_i[3] = { w / 2.0f, -w * sqrt(3.0f) / 2.0f, 0 };
  float bf_b2f_i[3] = { -w / 2.0f, w * sqrt(3.0f) / 2.0f, 0 };
  float cf_c1f_i[3] = { w / 2.0f, w * sqrt(3.0f) / 2.0f, 0 };
  float cf_c2f_i[3] = { -w / 2.0f, -w * sqrt(3.0f) / 2.0f, 0 };

  float af_a1f[3], af_a2f[3], bf_b1f[3], bf_b2f[3], cf_c1f[3], cf_c2f[3];
  rotations(af_a1f, af_a1f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(af_a2f, af_a2f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(bf_b1f, bf_b1f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(bf_b2f, bf_b2f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(cf_c1f, cf_c1f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(cf_c2f, cf_c2f_i, R_z_yaw, R_y_pitch, R_x_roll);

  float a1f[3] = { af[0] + af_a1f[0], af[1] + af_a1f[1], af[2] + af_a1f[2] };
  float a2f[3] = { af[0] + af_a2f[0], af[1] + af_a2f[1], af[2] + af_a2f[2] };
  float b1f[3] = { bf[0] + bf_b1f[0], bf[1] + bf_b1f[1], bf[2] + bf_b1f[2] };
  float b2f[3] = { bf[0] + bf_b2f[0], bf[1] + bf_b2f[1], bf[2] + bf_b2f[2] };
  float c1f[3] = { cf[0] + cf_c1f[0], cf[1] + cf_c1f[1], cf[2] + cf_c1f[2] };
  float c2f[3] = { cf[0] + cf_c2f[0], cf[1] + cf_c2f[1], cf[2] + cf_c2f[2] };

  float a1[3] = { a1f[0] - a10[0], a1f[1] - a10[1], a1f[2] - a10[2] };
  float a2[3] = { a2f[0] - a20[0], a2f[1] - a20[1], a2f[2] - a20[2] };
  float b1[3] = { b1f[0] - b10[0], b1f[1] - b10[1], b1f[2] - b10[2] };
  float b2[3] = { b2f[0] - b20[0], b2f[1] - b20[1], b2f[2] - b20[2] };
  float c1[3] = { c1f[0] - c10[0], c1f[1] - c10[1], c1f[2] - c10[2] };
  float c2[3] = { c2f[0] - c20[0], c2f[1] - c20[1], c2f[2] - c20[2] };

  // solve all six legs into a temporary array first
  float nt[6];
  bool ok = solveLeg(a1, na, a_10_20, +1, nt[0]) && solveLeg(a2, na, a_10_20, -1, nt[1]) && solveLeg(b1, nb, b_10_20, +1, nt[2]) && solveLeg(b2, nb, b_10_20, -1, nt[3]) && solveLeg(c1, nc, c_10_20, +1, nt[4]) && solveLeg(c2, nc, c_10_20, -1, nt[5]);

  if (!ok) {
    if (!PLOT) Serial.println("IK: pose not reachable (check hz_home / r_max) - keeping last pose");
    return false;
  }

  for (int i = 0; i < 6; i++) {
    nt[i] = -nt[i];  // negative = up
    if (isnan(nt[i]) || fabs(nt[i]) > 40) {
      if (!PLOT) {
        Serial.print("IK: servo ");
        Serial.print(ID[i]);
        Serial.print(" angle out of range: ");
        Serial.println(nt[i]);
      }
      return false;
    }
    if (fabs(nt[i] - theta[i]) > 80) {
      if (!PLOT) {
        Serial.print("IK: step too big on servo ");
        Serial.println(ID[i]);
      }
      return false;
    }
  }

  for (int i = 0; i < 6; i++) theta[i] = nt[i];
  return true;
}

void rotations(float* result, float hi[3], float Rz[3][3], float Ry[3][3], float Rx[3][3]) {
  float temp[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
  float combined[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        temp[i][j] += Rz[i][k] * Ry[k][j];

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        combined[i][j] += temp[i][k] * Rx[k][j];

  for (int i = 0; i < 3; i++) {
    result[i] = 0;
    for (int j = 0; j < 3; j++) result[i] += combined[i][j] * hi[j];
  }
}

float mag(const float array[]) {
  return sqrt(array[0] * array[0] + array[1] * array[1] + array[2] * array[2]);
}

float dot(const float array1[], const float array2[]) {
  return array1[0] * array2[0] + array1[1] * array2[1] + array1[2] * array2[2];
}
