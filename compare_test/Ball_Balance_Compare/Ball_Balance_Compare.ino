/*               ****NOTES****
  Maestro Control Center -> Serial Settings:
   Serial mode: UART, fixed baud rate
   Baud rate: 115200 (must match maestroSerial.begin below)
   CRC disabled
  Click "Apply Settings" after changes.

  Servo array indexes: a1=0 a2=1 b1=2 b2=3 c1=4 c2=5

  PID vs KALMAN+PID vs LQR COMPARISON SKETCH
  Copy of Ball_Balance_PID_log + Ball_Balance_LQR + Ball_Balance_KF_PID (those
  are unchanged). All controllers live in this one sketch and share EVERYTHING
  else (Pixy reading, tilt limit, inverse kinematics, servo speeds), so a
  comparison only shows the difference between the control laws:
      PID:     tilt = kp*e + kd*de/dt + ki_pid*int(e)        (de/dt: filtered difference)
      KF-PID:  tilt = kp*e^ + kd*(v^ - v_sp) + ki_pid*int(e^) (e^, v^: Kalman filter)
      LQR:     tilt = K[0]*e + K[1]*de/dt + ki_lqr*int(e),  K from the Riccati equation
      KF-LQR:  tilt = K[0]*e^ + K[1]*(v^ - v_sp) + ki_lqr*int(e^)
  PID / KF-PID and LQR / KF-LQR use the SAME gains, so each pair only differs in
  how position and velocity are estimated. The controller is switched at runtime, no re-flash
  needed between runs.

  SCRIPTED TEST (same for both controllers, set point is a pure function of
  the test time, so both runs get exactly the same reference):
    settle    hold centre (not used for metrics)
    out_AAA   step from centre to the edge point at angle AAA deg (radius ext_R)
    back_AAA  step back to the centre             (repeated for n_dirs angles)
    circle_in step to (circle_R, 0)
    circle    fast circle, radius circle_R, circle_T s per turn, circle_turns turns
    star_in   step to the top corner (0, star_R)
    star      pentagram, corner radius star_R, star_speed px/s, star_loops loops
    end       hold centre
  The workspace is a circle (set point limited to sp_max), so the "corners"
  are points on a circle of radius ext_R, every 360/n_dirs degrees.

  Serial commands (115200, end with newline):
    "m 0" ... "m 3"   controller PID / LQR / KF-PID / KF-LQR
    "r" or "r 4" or "r 4 1" run the test (optional: number of edge directions,
                      1..12; a second 1 keeps the servos on after the test ends,
                      so the next controller can start without touching the ball)
    "x"               abort the test
    "k kp kd ki"      new PID gains (positive)
    "l qp qv r"       new LQR weights, gains are recomputed
    "f qa r bu"       new Kalman settings (process noise px/s^2, meas. noise px^2,
                      bu 1/0 = use the tilt as input or not)
    "s x y" / "c"     manual set point / centre (only when no test is running)
    "e" / "o"         servos on (go home and balance) / off (limp)
    "?"               print the settings
  The servos start OFF and switch off again when a test ends or aborts and
  after idle_off s without a ball, so they do not draw current while waiting.
  Every text line starts with '#'. During a test one CSV row is printed per
  control step, after a header line starting with "t_ms".
*/

#include "PololuMaestro.h"
#include "math.h"
#include <Pixy2.h>

#define maestroSerial SERIAL_PORT_HARDWARE_OPEN  // for teensy

// ------------------- START TUNE -------------------

// ANGLE OFFSET FOR EACH MOTOR [NEGATIVE UP / POSITIVE DOWN] - DEGREES
float offset[6] = { -7, 0.0,
                    -2, -1,
                    0.0, 0.0 };

// CAMERA CALIBRATION
float origin[2] = { 157, 103 };
float r_platform = 124;

// SET POINT in px relative to the platform centre, +x right, +y up
float setpoint[2] = { 0, 0 };
float sp_max = 80;  // limit for the setpoint radius (px). Plate 205 x 205 mm: the edge is
                    // 102.5 mm = 98 px from the centre on the axes, keep a margin

// TEST SEQUENCE (short version, ~52 s per controller; the old long one was
// n_dirs 8, t_settle 4, t_step 5, t_entry 4, t_end 5, 3 turns, 2 loops = ~128 s)
int n_dirs = 4;            // edge points: 8 = every 45 deg, 4 = every 90 deg
float ext_R = 65;          // radius of the edge points (px, <= sp_max); was 70, the
                           // plate edge is 98 px out on the axes, so keep room for overshoot
float t_settle = 2;        // s, first centre hold
float t_step = 3;          // s, each hold after a step (out_/back_)
float t_entry = 2;         // s, hold before the circle and the star
float t_end = 4;           // s, last centre hold
float circle_R = 60;       // px
float circle_T = 4;        // s per turn (fast), negative = other direction
float circle_turns = 2;
float star_R = 60;         // px, corner radius
float star_speed = 60;     // px/s along the lines
float star_loops = 1;
float lost_abort = 1.0;    // s without ball during a test -> abort

// HOME pose. hz_home MUST be reachable: ~119..180 mm with p1=31, p2=150.
float hx_home = 0;
float hy_home = 0;
float hz_home = 145;
float roll_home = 0;
float pitch_home = 0;
float yaw_home = 0;

// MAX AVERAGE ANGLE (pitch/roll) IN ONE STEP - DEGREES
float r_max = 15;

// CONTROLLER: 0 = PID, 1 = LQR, 2 = KF-PID, 3 = KF-LQR (runtime: "m 0/1/2/3")
int ctrl = 0;

// PID (retuned 2026-11-12 from the 10:28 logs, was 0.09 / 0.05 / 0)
// ki added: the ball stopped 15-30 px short of the edge points with ~1.5 deg
// tilt held, a constant disturbance (plate not flat / friction) that only an
// integral removes. KF-PID uses the same gains.
float kp = 0.10;
float kd = 0.055;
float ki_pid = 0.08;

// LQR (retuned 2026-11-12, was q 1/1, r 100, dt 0.02, 1.24 px/mm, 5/7, ki 0)
// Model fixed to the measured plant: the logs give ~90-105 px/s^2 per deg,
// the old model said 151, so the old gains were designed for the wrong ball.
// New K ~ [0.123, 0.058] (old [0.085, 0.092]: too much velocity gain = noisy,
// it hit the tilt limit on the circle).
float q_pos = 1.0;       // penalty on position error
float q_vel = 0.05;      // penalty on velocity error
float r_lqr = 60.0;      // penalty on tilt angle (deg^2)
float dt_nom = 0.016;    // Pixy frame time (s): the logs run at 62 Hz
float g_mm = 9800.0;               // gravity (mm/s^2)
float px_per_mm = 1.0 / 1.045;     // camera px per mm: corner nuts (170 mm square, 120.2 mm
                                   // from the centre) are ~115 px out -> 1.045 mm/px
float ball_k = 0.614;              // 1/(1+I/mr^2) for the hollow 51 mm 12 g ball
float ki_lqr = 0.08;

float integral_max = 40;   // anti-windup clamp (pixel*s): ki*40 = 3.2 deg max, was 200
float i_zone = 35;         // px: the integral only runs when |error| < i_zone. During a
                           // big step it would wind up and throw the ball past the
                           // target (ball lost against a corner nut on 2026-11-12)

// KALMAN FILTER (KF-PID and KF-LQR, same as Ball_Balance_KF_PID / _KF_LQR). One filter per axis,
// state = [position px, velocity px/s], measurement = Pixy position.
//   predict:  p += v*dt + a*dt^2/2,  v += a*dt,  a = -kf_c * tilt (if kf_bu = 1)
//   update:   standard Kalman gain from P, Q (white acceleration noise), R
float kf_qa = 150;    // px/s^2, std of the acceleration the model does not know
float kf_r = 1.0;     // px^2, variance of the Pixy position (about 1 px std)
// ball acceleration per degree of tilt: g * px_per_mm / (1 + I/(m r^2)) * pi/180
// = 9800 mm/s^2 * (1/1.045 px/mm) * 0.614 (hollow 51 mm, 12 g ball) * 0.01745 ~ 100
float kf_c = 100;     // fitted from the 10:28 logs: 89-106 (model value 100)
int kf_bu = 1;        // 1 = use the sent tilt as known input, 0 = constant-velocity model
int kf_delay = 3;     // frames (~16 ms each) from sending a tilt until the ball feels it
float kf_gate = 30;   // px, a measurement this far from the prediction restarts the filter

// Derivative filter, both controllers: deriv_order low-pass stages in series,
// each with time constant T (s). 2 stages cut the Pixy pixel noise much more
// than the old single stage (T = 0.08) for about the same lag.
int deriv_order = 2;
float T = 0.05;

// Tilt deadband (deg): the servos only get a new target when the tilt command
// moved more than this since the last target sent. Stops the servos from
// reversing on every frame because of pixel noise (current / heat). 0 = off.
float tilt_db = 0.25;

// s without a ball (and no test running) before the servos are switched off
float idle_off = 20;

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

// **CONTROLLER STATE**
float error[2] = { 0, 0 };
float error_prev[2] = { 0, 0 };
float deriv[2] = { 0, 0 };
float deriv1[2] = { 0, 0 };    // first filter stage
float out_sent[2] = { 0, 0 };  // tilt of the last servo target sent
float integral[2] = { 0, 0 };
float out[2];
unsigned long time_i = 0;
bool tracking = false;  // true while controller state is valid
bool atHome = false;    // true once the home pose has been sent
bool servosOn = false;  // false = no servo pulses (limp)
unsigned long lastBall = 0;  // ms, last time the ball was seen

// **KALMAN STATE** (per axis)
const int KF_DMAX = 8;
struct KF { float p, v, P00, P01, P11; };
KF kf[2];
float u_hist[2][KF_DMAX];       // last tilts sent, u_hist[k][0] = newest
float sp_last[2] = { 0, 0 };    // set point at the previous control step
bool sp_jump = true;            // set point stepped -> no set point velocity this frame

// **LQR GAINS**  tilt[deg] = lqr_K[0]*error[px] + lqr_K[1]*error_velocity[px/s]
float lqr_K[2] = { 0, 0 };

// **TEST SEQUENCE STATE**
enum SegKind { SEG_HOLD = 0, SEG_CIRCLE = 1, SEG_STAR = 2 };
struct Segment {
  uint8_t kind;
  float x, y;  // HOLD: set point
  float R;     // CIRCLE / STAR: radius
  float p;     // CIRCLE: period (s), STAR: speed (px/s)
  float dur;   // s
  char name[12];
};
const int SEG_MAX = 32;
Segment seq[SEG_MAX];
int nSeg = 0;
bool testRunning = false;
bool keepOn = false;          // "r n 1": servos stay on after a test that ended normally
int curSeg = 0;
unsigned long testStart = 0;  // ms
float segStart = 0;           // s, test time at which curSeg started
unsigned long lastSeen = 0;   // ms, last time the ball was seen during the test

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

  // sanity check of the home pose (servos stay off until "e")
  if (!InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home)) {
    Serial.println("# WARNING: home pose not reachable! Change hz_home (approx. 119..180 mm).");
  }
  servosOff();
}

// target 0 = the Maestro stops sending pulses, the servos go limp
void servosOff() {
  for (int i = 0; i < 6; i++) maestro.setTarget(i, 0);
  servosOn = false;
  atHome = false;
  tracking = false;
  Serial.println("# SERVOS OFF");
}

void servosEnable() {
  servosOn = true;
  atHome = false;
  lastBall = millis();
  goHome();
  Serial.println("# SERVOS ON");
}

// Step the set point. Shifts the stored previous error by the same amount so a
// step does not cause a derivative kick, and clears the integral.
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
  sp_jump = true;
}

// ---------------- LQR DESIGN (unchanged from Ball_Balance_LQR) ----------------
// One axis, discrete time with step dt:
//   x = [pos, vel],  x+ = A x + B u,   A = [[1, dt], [0, 1]],  B = [-c*dt^2/2, -c*dt]
// c = ball acceleration per degree of tilt (px/s^2/deg).
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
  float w0 = p00 * b0 + p01 * b1;
  float w1 = p01 * b0 + p11 * b1;
  S = r_lqr + b0 * w0 + b1 * w1;
  v0 = w0;
  v1 = dt * w0 + w1;
  lqr_K[0] = -v0 / S;
  lqr_K[1] = -v1 / S;
}

const char* ctrlName() {
  static const char* names[4] = { "PID", "LQR", "KF-PID", "KF-LQR" };
  return names[constrain(ctrl, 0, 3)];
}

// ---------------- KALMAN FILTER ----------------
void kfInit(int k, float z) {
  kf[k].p = z;
  kf[k].v = 0;
  kf[k].P00 = kf_r;
  kf[k].P01 = 0;
  kf[k].P11 = 2500;  // (50 px/s)^2: the ball may already be rolling
  for (int i = 0; i < KF_DMAX; i++) u_hist[k][i] = 0;
}

// one predict + update step for axis k, measurement z (px), step dt (s)
void kfStep(int k, float z, float dt) {
  KF &f = kf[k];
  int d = constrain(kf_delay, 0, KF_DMAX - 1);
  float a = kf_bu ? -kf_c * u_hist[k][d] : 0;  // positive tilt pushes the ball to -
  // predict
  f.p += f.v * dt + 0.5f * a * dt * dt;
  f.v += a * dt;
  float q = kf_qa * kf_qa;
  float dt2 = dt * dt;
  float P00 = f.P00 + 2 * dt * f.P01 + dt2 * f.P11 + q * dt2 * dt2 / 4;
  float P01 = f.P01 + dt * f.P11 + q * dt2 * dt / 2;
  float P11 = f.P11 + q * dt2;
  // update
  float innov = z - f.p;
  if (fabs(innov) > kf_gate) {  // jump (wrong block / ball picked up): restart
    kfInit(k, z);
    return;
  }
  float S = P00 + kf_r;
  float K0 = P00 / S;
  float K1 = P01 / S;
  f.p += K0 * innov;
  f.v += K1 * innov;
  f.P00 = (1 - K0) * P00;
  f.P01 = (1 - K0) * P01;
  f.P11 = P11 - K1 * P01;
}

// remember the tilt that was just applied (newest first)
void kfPushTilt(float ux, float uy) {
  for (int i = KF_DMAX - 1; i > 0; i--) {
    u_hist[x][i] = u_hist[x][i - 1];
    u_hist[y][i] = u_hist[y][i - 1];
  }
  u_hist[x][0] = ux;
  u_hist[y][0] = uy;
}

void printSettings() {
  Serial.print("# CTRL ");    Serial.println(ctrlName());
  Serial.print("# PID kp=");  Serial.print(kp, 5);
  Serial.print(" kd=");       Serial.print(kd, 5);
  Serial.print(" ki=");       Serial.println(ki_pid, 5);
  Serial.print("# LQR q_pos="); Serial.print(q_pos, 4);
  Serial.print(" q_vel=");    Serial.print(q_vel, 4);
  Serial.print(" r=");        Serial.print(r_lqr, 4);
  Serial.print(" Kpos=");     Serial.print(lqr_K[0], 5);
  Serial.print(" Kvel=");     Serial.print(lqr_K[1], 5);
  Serial.print(" ki=");       Serial.println(ki_lqr, 5);
  Serial.print("# KF qa=");   Serial.print(kf_qa, 2);
  Serial.print(" r=");        Serial.print(kf_r, 3);
  Serial.print(" c=");        Serial.print(kf_c, 2);
  Serial.print(" bu=");       Serial.print(kf_bu);
  Serial.print(" delay=");    Serial.print(kf_delay);
  Serial.print(" gate=");     Serial.println(kf_gate, 1);
  Serial.print("# COMMON T="); Serial.print(T, 4);
  Serial.print(" r_max=");    Serial.print(r_max, 2);
  Serial.print(" sp_max=");   Serial.print(sp_max, 1);
  Serial.print(" deriv_order="); Serial.print(deriv_order);
  Serial.print(" tilt_db=");  Serial.print(tilt_db, 3);
  Serial.print(" i_zone=");   Serial.print(i_zone, 1);
  Serial.print(" i_max=");    Serial.println(integral_max, 1);
}

// ---------------- TEST SEQUENCE ----------------
void addSeg(uint8_t kind, float sx, float sy, float R, float p, float dur, const char* name) {
  if (nSeg >= SEG_MAX) return;
  Segment &s = seq[nSeg++];
  s.kind = kind;
  s.x = sx;
  s.y = sy;
  s.R = R;
  s.p = p;
  s.dur = dur;
  strncpy(s.name, name, sizeof(s.name) - 1);
  s.name[sizeof(s.name) - 1] = 0;
}

float starChord() { return 2.0f * star_R * sin(72.0f * toRad); }  // one line of the star

void buildSequence() {
  nSeg = 0;
  float R = constrain(ext_R, 0, sp_max);
  addSeg(SEG_HOLD, 0, 0, 0, 0, t_settle, "settle");
  char name[12];
  for (int k = 0; k < n_dirs; k++) {
    int deg = (int)lround(360.0f * k / n_dirs);
    float a = deg * toRad;
    snprintf(name, sizeof(name), "out_%03d", deg);
    addSeg(SEG_HOLD, R * cos(a), R * sin(a), 0, 0, t_step, name);
    snprintf(name, sizeof(name), "back_%03d", deg);
    addSeg(SEG_HOLD, 0, 0, 0, 0, t_step, name);
  }
  float cR = constrain(circle_R, 0, sp_max);
  addSeg(SEG_HOLD, cR, 0, 0, 0, t_entry, "circle_in");
  addSeg(SEG_CIRCLE, 0, 0, cR, circle_T, circle_turns * fabs(circle_T), "circle");
  star_R = constrain(star_R, 10, sp_max);
  addSeg(SEG_HOLD, 0, star_R, 0, 0, t_entry, "star_in");
  addSeg(SEG_STAR, 0, 0, star_R, star_speed, star_loops * 5.0f * starChord() / star_speed, "star");
  addSeg(SEG_HOLD, 0, 0, 0, 0, t_end, "end");
}

// corner k of the pentagram, visited in the order 0,2,4,1,3 (first at the top)
void starVertex(int k, float R, float &vx, float &vy) {
  static const int order[5] = { 0, 2, 4, 1, 3 };
  float a = (90.0f + 72.0f * order[k % 5]) * toRad;
  vx = R * cos(a);
  vy = R * sin(a);
}

// set point of segment s, ts seconds after the segment started
void segSetpoint(const Segment &s, float ts, float &sx, float &sy) {
  if (s.kind == SEG_CIRCLE) {
    float a = 2.0f * PI * ts / s.p;  // starts at (R, 0)
    sx = s.R * cos(a);
    sy = s.R * sin(a);
  } else if (s.kind == SEG_STAR) {
    float L = 2.0f * s.R * sin(72.0f * toRad);
    float d = ts * s.p;
    int i = (int)floor(d / L);
    float f = d / L - i;
    float ax, ay, bx, by;
    starVertex(i, s.R, ax, ay);
    starVertex(i + 1, s.R, bx, by);
    sx = ax + (bx - ax) * f;
    sy = ay + (by - ay) * f;
  } else {
    sx = s.x;
    sy = s.y;
  }
}

void printSegment(int i) {
  const Segment &s = seq[i];
  Serial.print("# SEG ");   Serial.print(i);
  Serial.print(' ');        Serial.print(s.name);
  Serial.print(" kind=");   Serial.print(s.kind);
  Serial.print(" x=");      Serial.print(s.x, 2);
  Serial.print(" y=");      Serial.print(s.y, 2);
  Serial.print(" R=");      Serial.print(s.R, 2);
  Serial.print(" p=");      Serial.print(s.p, 3);
  Serial.print(" dur=");    Serial.println(s.dur, 3);
}

void startTest() {
  if (!servosOn) {
    Serial.println("# ERROR servos off - send e first");
    return;
  }
  if (!ballFound) {
    Serial.println("# ERROR no ball - place the ball on the platform first");
    return;
  }
  buildSequence();
  setSetpoint(0, 0);
  Serial.println("# START");
  printSettings();
  for (int i = 0; i < nSeg; i++) printSegment(i);
  Serial.println("t_ms,seg,sp_x,sp_y,ball_x,ball_y,err_x,err_y,derr_x,derr_y,u_x,u_y,sat,found,ik");
  curSeg = 0;
  segStart = 0;
  testStart = millis();
  lastSeen = testStart;
  testRunning = true;
}

void stopTest(const char* why) {
  testRunning = false;
  setSetpoint(0, 0);
  Serial.print("# ");
  Serial.println(why);
  // after a normal END with keepOn the ball is held at the centre for the next run
  if (!(keepOn && strcmp(why, "END") == 0)) servosOff();
  else lastBall = millis();
}

// Moves the set point along the sequence. Called every loop, before the error
// is computed. A jump between hold segments is a STEP: it goes through
// setSetpoint() (no derivative kick, like the "s" command in the old sketches).
// Circle and star write the set point directly, so the error derivative is
// (ball velocity - set point velocity), which is what we want when tracking.
void updateTest() {
  if (!testRunning) return;
  float tt = (millis() - testStart) / 1000.0f;
  bool newSeg = false;
  while (curSeg < nSeg && tt - segStart >= seq[curSeg].dur) {
    segStart += seq[curSeg].dur;
    curSeg++;
    newSeg = true;
  }
  if (curSeg >= nSeg) {
    stopTest("END");
    return;
  }
  float sx, sy;
  segSetpoint(seq[curSeg], tt - segStart, sx, sy);
  if (newSeg && seq[curSeg].kind == SEG_HOLD) {
    setSetpoint(sx, sy);
  } else {
    setpoint[x] = sx;
    setpoint[y] = sy;
  }
}

// one CSV row (found: 1 = ball seen, 0 = not seen, 2 = outside the platform)
void logRow(float bx, float by, int sat, int found, int ik) {
  Serial.print(millis() - testStart);  Serial.print(',');
  Serial.print(curSeg);                Serial.print(',');
  Serial.print(setpoint[x], 2);        Serial.print(',');
  Serial.print(setpoint[y], 2);        Serial.print(',');
  Serial.print(bx, 1);                 Serial.print(',');
  Serial.print(by, 1);                 Serial.print(',');
  Serial.print(error[x], 2);           Serial.print(',');
  Serial.print(error[y], 2);           Serial.print(',');
  Serial.print(deriv[x], 2);           Serial.print(',');
  Serial.print(deriv[y], 2);           Serial.print(',');
  Serial.print(out_sent[x], 3);        Serial.print(',');
  Serial.print(out_sent[y], 3);        Serial.print(',');
  Serial.print(sat);                   Serial.print(',');
  Serial.print(found);                 Serial.print(',');
  Serial.println(ik);
}

void handleSerial() {
  if (!Serial.available()) return;
  char cmd = Serial.peek();
  if (cmd == '\n' || cmd == '\r' || cmd == ' ') {  // ignore line endings / spaces
    Serial.read();
    return;
  }
  Serial.read();  // consume the command letter

  if (cmd == 'x') {
    if (testRunning) stopTest("ABORT by user");
  } else if (testRunning) {
    Serial.println("# BUSY test running, send x to abort");
  } else if (cmd == 'r') {
    int n = (int)Serial.parseFloat();
    int keep = (int)Serial.parseFloat();  // 0 if not sent
    if (n >= 1 && n <= 12) n_dirs = n;
    keepOn = (keep == 1);
    startTest();
  } else if (cmd == 'm') {
    int c = (int)Serial.parseFloat();
    ctrl = (c >= 1 && c <= 3) ? c : 0;
    tracking = false;  // restart the controller state
    printSettings();
  } else if (cmd == 'k') {
    float a = Serial.parseFloat();
    float b = Serial.parseFloat();
    float c = Serial.parseFloat();
    if (a > 0 && b >= 0 && c >= 0) {
      kp = a;
      kd = b;
      ki_pid = c;
    }
    printSettings();
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
    printSettings();
  } else if (cmd == 'f') {
    float qa = Serial.parseFloat();
    float rr = Serial.parseFloat();
    int bu = (int)Serial.parseFloat();
    if (qa > 0 && rr > 0) {
      kf_qa = qa;
      kf_r = rr;
      kf_bu = (bu == 1) ? 1 : 0;
    }
    printSettings();
  } else if (cmd == 's') {
    float sx = Serial.parseFloat();
    float sy = Serial.parseFloat();
    setSetpoint(sx, sy);
  } else if (cmd == 'c') {
    setSetpoint(0, 0);
  } else if (cmd == 'e') {
    servosEnable();
  } else if (cmd == 'o') {
    servosOff();
  } else if (cmd == '?') {
    printSettings();
  }
  while (Serial.available()) Serial.read();  // drop the rest of the line
}

void loop() {
  handleSerial();
  updateTest();
  findBall();
  if (ballFound) lastBall = millis();
  if (!servosOn) return;
  if (!testRunning && (millis() - lastBall) / 1000.0f > idle_off) {
    servosOff();
    return;
  }
  if (!ballFound) {
    tracking = false;  // reset controller memory
    if (!atHome) goHome();
    if (testRunning) {
      logRow(0, 0, 0, 0, 1);
      if ((millis() - lastSeen) / 1000.0f > lost_abort) stopTest("ABORT ball lost");
    }
  } else {
    if (testRunning) lastSeen = millis();
    control();
  }
}

void goHome() {
  if (!servosOn) return;
  if (InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home)) {
    moveServos(vel_home, acc_home);
    atHome = true;
    out_sent[x] = out_sent[y] = 0;
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
  if (!servosOn) return;
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
  ball[x] = pixy.ccc.blocks[best].m_x;
  ball[y] = pixy.ccc.blocks[best].m_y;
  ballFound = true;
}

void control() {
  // ball position relative to the platform centre (y flipped: camera y is down)
  float pos_x = ball[x] - origin[x];
  float pos_y = -(ball[y] - origin[y]);

  // error = ball position - set point
  error[x] = pos_x - setpoint[x];
  error[y] = pos_y - setpoint[y];

  // ball outside the platform (measured from the centre, not the set point) -> home
  float r_ball = sqrt(pos_x * pos_x + pos_y * pos_y);
  if (r_ball > r_platform) {
    tracking = false;
    if (!atHome) goHome();
    if (testRunning) logRow(pos_x, pos_y, 0, 2, 1);
    return;
  }

  unsigned long now = millis();

  // (re)start tracking: initialise state, skip derivative for this frame
  if (!tracking) {
    error_prev[x] = error[x];
    error_prev[y] = error[y];
    deriv[x] = deriv[y] = 0;
    deriv1[x] = deriv1[y] = 0;
    integral[x] = integral[y] = 0;
    kfInit(x, pos_x);
    kfInit(y, pos_y);
    sp_last[x] = setpoint[x];
    sp_last[y] = setpoint[y];
    sp_jump = false;
    time_i = now;
    tracking = true;
    return;
  }

  float dt = (now - time_i) / 1000.0f;
  if (dt < 0.001f) return;  // too fast, wait for the next frame
  time_i = now;

  bool lqr = (ctrl == 1 || ctrl == 3);
  float k_e = lqr ? lqr_K[0] : kp;
  float k_d = lqr ? lqr_K[1] : kd;
  float k_i = lqr ? ki_lqr : ki_pid;

  if (ctrl == 2 || ctrl == 3) {
    // KF-PID / KF-LQR: position term and integral on the filtered position,
    // velocity term on (ball velocity - set point velocity), both from the Kalman filter
    float z[2] = { pos_x, pos_y };
    for (int k = 0; k < 2; k++) {
      kfStep(k, z[k], dt);
      float sp_v = sp_jump ? 0 : (setpoint[k] - sp_last[k]) / dt;  // circle / star speed
      sp_last[k] = setpoint[k];
      error[k] = kf[k].p - setpoint[k];
      deriv[k] = kf[k].v - sp_v;
      if (fabs(error[k]) < i_zone)  // conditional integration (anti-windup)
        integral[k] += (error[k] + error_prev[k]) * dt / 2.0f;
      integral[k] = constrain(integral[k], -integral_max, integral_max);
      error_prev[k] = error[k];
      out[k] = k_e * error[k] + k_d * deriv[k] + k_i * integral[k];
    }
    sp_jump = false;
  } else {
    for (int k = 0; k < 2; k++) {
      float raw = (error[k] - error_prev[k]) / dt;
      // low-pass filtered derivative, 1 or 2 stages
      deriv1[k] += dt / T * (raw - deriv1[k]);
      if (deriv_order >= 2) deriv[k] += dt / T * (deriv1[k] - deriv[k]);
      else deriv[k] = deriv1[k];
      if (isnan(deriv[k]) || isinf(deriv[k]) || isnan(deriv1[k]) || isinf(deriv1[k])) deriv[k] = deriv1[k] = 0;

      if (fabs(error[k]) < i_zone)  // conditional integration (anti-windup)
        integral[k] += (error[k] + error_prev[k]) * dt / 2.0f;  // trapezoidal
      integral[k] = constrain(integral[k], -integral_max, integral_max);

      error_prev[k] = error[k];
      out[k] = k_e * error[k] + k_d * deriv[k] + k_i * integral[k];
    }
  }

  // limit the tilt
  int sat = 0;
  float r_out = sqrt(out[x] * out[x] + out[y] * out[y]);
  if (r_out > r_max) {
    out[x] *= r_max / r_out;
    out[y] *= r_max / r_out;
    sat = 1;
  }

  // deadband: only send a new target when the tilt really changed
  bool ik = true;
  if (fabs(out[x] - out_sent[x]) > tilt_db || fabs(out[y] - out_sent[y]) > tilt_db) {
    // out[x] -> pitch(+), out[y] -> roll(-)
    ik = InverseKinematics(hx_home, hy_home, hz_home, roll_home - out[y], pitch_home + out[x], yaw_home);
    if (ik) {
      moveServos(vel_control, acc_control);
      out_sent[x] = out[x];
      out_sent[y] = out[y];
      atHome = false;
    }
  }
  kfPushTilt(out_sent[x], out_sent[y]);  // the tilt the platform is really going to
  if (testRunning) logRow(pos_x, pos_y, sat, 1, ik ? 1 : 0);
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
    if (!testRunning) Serial.println("# IK: pose not reachable (check hz_home / r_max) - keeping last pose");
    return false;
  }

  for (int i = 0; i < 6; i++) {
    nt[i] = -nt[i];  // negative = up
    if (isnan(nt[i]) || fabs(nt[i]) > 40) {
      if (!testRunning) {
        Serial.print("# IK: servo ");
        Serial.print(ID[i]);
        Serial.print(" angle out of range: ");
        Serial.println(nt[i]);
      }
      return false;
    }
    if (fabs(nt[i] - theta[i]) > 80) {
      if (!testRunning) {
        Serial.print("# IK: step too big on servo ");
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
