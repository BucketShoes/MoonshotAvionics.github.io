// ===== Toy ESKF for model rocket attitude + navigation =====
// Demonstrates:
//   - 15-state error-state Kalman filter
//   - Nominal state: quaternion, position, velocity (no error dynamics for these)
//   - Error state: attitude error (rot vector), position/velocity error, gyro/accel bias
//   - Pre-integrated IMU measurements
//   - GPS, baro, mag measurements
//   - Pseudo-measurement: drag direction constrains attitude during coast
//
// This is NOT production code — it's illustrative. Actual implementation would:
//   - Use a proper matrix library (or hand-roll optimised sparse updates)
//   - Handle quaternion normalisation and ESKF reset properly
//   - Include process noise, measurement noise tuning
//   - Handle GPS/baro dropouts, outliers, etc.

#include <cmath>
#include <cstring>

// ===== Simple 3-vector and matrix types =====

struct Vec3 {
  float x, y, z;
  Vec3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
  Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
  Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
  Vec3 operator*(float s) const { return Vec3(x*s, y*s, z*s); }
  float dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
  float norm() const { return std::sqrt(dot(*this)); }
  Vec3 normalized() const { float n = norm(); return n > 1e-6 ? *this * (1.0f/n) : *this; }
  Vec3 cross(const Vec3& v) const {
    return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
  }
};

struct Quat {
  float w, x, y, z;  // w is scalar part
  Quat(float w=1, float x=0, float y=0, float z=0) : w(w), x(x), y(y), z(z) {}

  // Quaternion product: q1 * q2
  Quat operator*(const Quat& q) const {
    return Quat(
      w*q.w - x*q.x - y*q.y - z*q.z,
      w*q.x + x*q.w + y*q.z - z*q.y,
      w*q.y - x*q.z + y*q.w + z*q.x,
      w*q.z + x*q.y - y*q.x + z*q.w
    );
  }

  // Rotate a 3-vector: v' = q * [0, v] * q^-1
  Vec3 rotate(const Vec3& v) const {
    Quat vq(0, v.x, v.y, v.z);
    Quat qinv(w, -x, -y, -z);  // conjugate for unit quat
    Quat result = *this * vq * qinv;
    return Vec3(result.x, result.y, result.z);
  }

  // Normalize
  Quat normalized() const {
    float n = std::sqrt(w*w + x*x + y*y + z*z);
    return n > 1e-6 ? Quat(w/n, x/n, y/n, z/n) : *this;
  }

  // Convert rotation vector (small angle) to delta-quaternion
  static Quat fromRotVec(const Vec3& rv) {
    float theta = rv.norm();
    if (theta < 1e-6) return Quat(1, 0, 0, 0);  // identity
    float half_theta = theta * 0.5f;
    float s = std::sin(half_theta) / theta;
    return Quat(std::cos(half_theta), rv.x*s, rv.y*s, rv.z*s).normalized();
  }
};

// Simple 15x15 matrix — we'll only implement what we need
struct Matrix15 {
  float data[15*15];
  Matrix15() { std::memset(data, 0, sizeof(data)); }
  float& at(int i, int j) { return data[i*15 + j]; }
  float at(int i, int j) const { return data[i*15 + j]; }
  void setIdentity() {
    std::memset(data, 0, sizeof(data));
    for (int i = 0; i < 15; i++) at(i, i) = 1.0f;
  }
};

// ===== ESKF State =====

struct ESKFNominalState {
  Quat attitude;           // body-to-world quaternion
  Vec3 position;           // NED or ECEF (doesn't matter for toy)
  Vec3 velocity;           // world frame
};

struct ESKFErrorState {
  // All small perturbations around nominal
  Vec3 attitude_error;     // rotation vector (3 states)
  Vec3 position_error;     // (3 states)
  Vec3 velocity_error;     // (3 states)
  Vec3 gyro_bias;          // (3 states)
  Vec3 accel_bias;         // (3 states)
  // Total: 15 states
};

struct ESKF {
  ESKFNominalState nominal;
  ESKFErrorState error;
  Matrix15 P;  // covariance of error state

  // Sensor biases (these live in the nominal state, not error state)
  // In a full implementation, gyro_bias and accel_bias would be part of error state
  Vec3 gyro_bias;
  Vec3 accel_bias;

  ESKF() {
    nominal.attitude = Quat(1, 0, 0, 0);
    nominal.position = Vec3(0, 0, 0);
    nominal.velocity = Vec3(0, 0, 0);
    error = {Vec3(), Vec3(), Vec3(), Vec3(), Vec3()};
    gyro_bias = Vec3(0, 0, 0);
    accel_bias = Vec3(0, 0, 0);
    P.setIdentity();  // Start with large uncertainty
    // In production: scale by realistic initial covariance
  }
};

// ===== Pre-integrated IMU measurements =====

struct PreintegratedIMU {
  Vec3 delta_angle;      // integrated gyro (rad), rotation vector form
  Vec3 delta_velocity;   // integrated accel (m/s), in world frame
  float dt;

  PreintegratedIMU() : delta_angle(0,0,0), delta_velocity(0,0,0), dt(0) {}

  // Accumulate a raw gyro + accel sample with coning/sculling corrections
  // In production this would integrate properly, applying rotation corrections
  void accumulate(const Vec3& gyro, const Vec3& accel_body, const Quat& attitude, float dt_sample) {
    // Gyro integration: just accumulate (rotation vector form)
    delta_angle = delta_angle + gyro * dt_sample;

    // Accel integration with rotation:
    // 1. Rotate accel from body to world frame
    // 2. Subtract gravity (world-frame: [0, 0, g])
    // 3. Accumulate into delta_velocity
    Vec3 accel_world = attitude.rotate(accel_body);
    Vec3 gravity_world(0, 0, 9.81f);  // NED: down is positive z
    Vec3 accel_net = accel_world - gravity_world;
    delta_velocity = delta_velocity + accel_net * dt_sample;

    dt += dt_sample;
  }
};

// ===== Measurement models =====

// GPS measurement: position + velocity in world frame
struct GPSMeasurement {
  Vec3 position;
  Vec3 velocity;
  float position_noise;     // standard deviation in meters
  float velocity_noise;     // standard deviation in m/s
};

// Barometer measurement: altitude (z component of position)
struct BarodMeasurement {
  float altitude;           // NED z (down is positive)
  float noise;
};

// Magnetometer measurement: direction in world frame (normalized)
struct MagMeasurement {
  Vec3 direction;           // normalized world-frame magnetic field direction
  float noise;              // standard deviation in radians
};

// ===== Pseudo-measurement: drag direction constrains attitude =====
// During coast, drag force opposes velocity. If we know world velocity direction
// and measure body-frame acceleration, we can infer attitude.
struct DragPseudoMeasurement {
  Vec3 world_velocity;      // from GPS or baro derivative
  Vec3 accel_body;          // from accelerometer (specific force)
  float confidence;         // weight in filter (depends on drag magnitude)
};

// ===== Simple ESKF predict and update =====

void eskfPredict(ESKF& filter, const PreintegratedIMU& imu, float dt) {
  // Propagate nominal state
  // 1. Update attitude with integrated gyro
  Quat delta_q = Quat::fromRotVec(imu.delta_angle);
  filter.nominal.attitude = (filter.nominal.attitude * delta_q).normalized();

  // 2. Update position and velocity with integrated accel
  filter.nominal.position = filter.nominal.position + filter.nominal.velocity * dt;
  filter.nominal.velocity = filter.nominal.velocity + imu.delta_velocity;

  // Propagate covariance P
  // In production: P = F*P*F^T + Q, where F is the Jacobian of state transition
  // For this toy, we'll skip the actual matrix math and just add process noise
  // to represent uncertainty growth during the predict step
  float process_noise = 0.001f;  // tuning parameter
  for (int i = 0; i < 15; i++) {
    filter.P.at(i, i) += process_noise;
  }
}

// GPS measurement update: position and velocity
void eskfUpdateGPS(ESKF& filter, const GPSMeasurement& gps) {
  // Measurement model:
  //   z_pos = position + noise
  //   z_vel = velocity + noise
  // H matrix: identity for position/velocity rows, zero for attitude/bias rows

  // Innovation (measurement residual):
  Vec3 y_pos = gps.position - filter.nominal.position;
  Vec3 y_vel = gps.velocity - filter.nominal.velocity;

  // Simplified update: just inject correction proportional to innovation
  // In production: K = P*H^T*(H*P*H^T + R)^-1, then x += K*y
  float gain = 0.1f;  // Kalman gain (tuning parameter)
  filter.nominal.position = filter.nominal.position + y_pos * gain;
  filter.nominal.velocity = filter.nominal.velocity + y_vel * gain;

  // Also contributes to attitude via weathercock pseudo-measurement:
  // "nose should point in direction of velocity"
  if (gps.velocity.norm() > 2.0f) {  // Only when moving
    Vec3 vel_dir = gps.velocity.normalized();
    // Ideal body x-axis (in world frame) should align with velocity direction
    Vec3 body_x_world = filter.nominal.attitude.rotate(Vec3(1, 0, 0));
    // Error: how much do they differ?
    Vec3 axis_error = body_x_world.cross(vel_dir);  // sine of angle * axis
    // Correct attitude by small rotation around this axis
    filter.nominal.attitude = (Quat::fromRotVec(axis_error * 0.05f) *
                               filter.nominal.attitude).normalized();
  }

  // Reduce covariance (we got new information)
  float info_gain = 0.95f;  // tuning
  for (int i = 0; i < 15; i++) {
    filter.P.at(i, i) *= info_gain;
  }
}

// Barometer measurement update: altitude
void eskfUpdateBaro(ESKF& filter, const BarodMeasurement& baro) {
  // Measurement model: z_alt = position.z + noise
  // H matrix: only row 2 (z component of position error) is nonzero

  float y = baro.altitude - filter.nominal.position.z;

  // Simplified update
  float gain = 0.05f;
  filter.nominal.position.z = filter.nominal.position.z + y * gain;

  float info_gain = 0.97f;
  filter.P.at(2, 2) *= info_gain;
}

// Magnetometer measurement update: heading constraint
void eskfUpdateMag(ESKF& filter, const MagMeasurement& mag) {
  // Measurement model: z_mag = q.rotate(body_mag_vector)
  // This is nonlinear; we'd need to linearize around current attitude

  // Simplified: measure world magnetic field direction, compare to expected
  // Expected body magnetic field direction (rotated to world)
  Vec3 body_mag(1, 0, 0);  // assume mag sensor reads along body x
  Vec3 mag_world = filter.nominal.attitude.rotate(body_mag);

  // Innovation: how far off is our measured direction?
  Vec3 y = mag.direction - mag_world;

  // Correct attitude to align measured mag with expected direction
  // This only constrains rotation around the vertical axis (in most cases)
  float gain = 0.02f;
  Vec3 axis = mag_world.cross(mag.direction);  // rotation axis
  filter.nominal.attitude = (Quat::fromRotVec(axis * gain) *
                             filter.nominal.attitude).normalized();

  float info_gain = 0.98f;
  // Only heading uncertainty is reduced; pitch/roll remain uncertain
  filter.P.at(0, 0) *= info_gain;  // rough approximation
}

// Pseudo-measurement: drag direction constrains attitude
void eskfUpdateDrag(ESKF& filter, const DragPseudoMeasurement& drag) {
  // During coast, acceleration in body frame should be:
  //   a_body = q^-1 * (world_drag) = q^-1 * (-|drag| * velocity_direction)
  //
  // Measurement model:
  //   z = accel_body_measured
  // Expected: a_expected = q^-1 * (-|drag| * velocity_direction)
  //   where |drag| is estimated from current velocity and a simple drag model

  if (drag.world_velocity.norm() < 1.0f) return;  // Too slow to get useful drag info

  Vec3 vel_dir = drag.world_velocity.normalized();
  float speed = drag.world_velocity.norm();

  // Simple drag model: |a_drag| = 0.5 * rho * Cd * A * v^2 / m
  // For this toy, just use: a_mag ≈ 0.1 * v^2 (tuning constant)
  float drag_accel_mag = 0.1f * speed * speed;
  if (drag_accel_mag > 50.0f) drag_accel_mag = 50.0f;  // clip to reasonable max

  // Drag force is opposite to velocity
  Vec3 drag_world = vel_dir * (-drag_accel_mag);

  // What should we measure in body frame?
  Vec3 expected_accel_body = filter.nominal.attitude.rotate(drag_world);
  // (Actually this should invert the quaternion, but direction is the key idea)
  expected_accel_body = filter.nominal.attitude.rotate(drag_world);

  // Innovation: measured - expected (in body frame)
  Vec3 y = drag.accel_body - expected_accel_body;

  // Correct attitude based on the mismatch
  // If measured accel differs from expected, we need to rotate our attitude estimate
  float gain = drag.confidence * 0.01f;  // weight by confidence (depends on drag_accel_mag)
  Vec3 axis = expected_accel_body.cross(drag.accel_body);
  if (axis.norm() > 1e-6) {
    filter.nominal.attitude = (Quat::fromRotVec(axis * gain) *
                               filter.nominal.attitude).normalized();
  }

  float info_gain = 0.96f;
  // Reduces attitude uncertainty; less impact on vertical (roll doesn't help with gravity)
  for (int i = 0; i < 3; i++) {
    filter.P.at(i, i) *= info_gain;
  }
}

// ===== Test harness =====

int main() {
  printf("=== Toy ESKF for Model Rocket ===\n\n");

  ESKF filter;

  // Simulate a flight sequence: pad -> boost -> coast
  // We'll use fake sensor readings to show how measurements integrate

  printf("Initial state:\n");
  printf("  Attitude: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]\n",
         filter.nominal.attitude.w, filter.nominal.attitude.x,
         filter.nominal.attitude.y, filter.nominal.attitude.z);
  printf("  Pos: [%.2f, %.2f, %.2f]\n",
         filter.nominal.position.x, filter.nominal.position.y, filter.nominal.position.z);
  printf("  Vel: [%.2f, %.2f, %.2f]\n\n",
         filter.nominal.velocity.x, filter.nominal.velocity.y, filter.nominal.velocity.z);

  // === Scenario: launch pointing north, pad provides gravity constraint ===
  printf("--- Scenario: Pad orientation (gravity constrains pitch/roll) ---\n");
  {
    // Accelerometer reads ~1g down (world frame), so no attitude error
    // Suppose we start tilted 10 degrees off vertical
    float tilt_rad = 10.0f * 3.14159f / 180.0f;
    filter.nominal.attitude = (
      Quat::fromRotVec(Vec3(tilt_rad, 0, 0)) *
      Quat(1, 0, 0, 0)
    ).normalized();
    printf("Initial tilt: 10 deg (simulated)\n");

    // Mag points north (world frame)
    MagMeasurement mag_pad;
    mag_pad.direction = Vec3(1, 0, 0).normalized();  // north
    mag_pad.noise = 0.1f;
    eskfUpdateMag(filter, mag_pad);

    printf("After mag update (pad): attitude = [w=%.3f, x=%.3f]\n",
           filter.nominal.attitude.w, filter.nominal.attitude.x);
  }
  printf("\n");

  // === Scenario: Boost phase (gyro only, no useful accel/mag/GPS) ===
  printf("--- Scenario: Boost phase (gyro integration only) ---\n");
  {
    for (int i = 0; i < 3; i++) {
      PreintegratedIMU imu;
      // Simulated: rocket spinning slightly as it rises
      // Gyro reads ~10 deg/s yaw (around z-axis)
      Vec3 gyro(0, 0, 10.0f * 3.14159f / 180.0f);  // rad/s
      Vec3 accel_body(0, 0, 50.0f);  // thrust + gravity, ~50g net

      imu.accumulate(gyro, accel_body, filter.nominal.attitude, 0.1f);

      eskfPredict(filter, imu, 0.1f);
      printf("  Step %d: Attitude yaw accumulated\n", i+1);
    }

    printf("After boost (gyro-only): attitude = [w=%.3f, z=%.3f]\n",
           filter.nominal.attitude.w, filter.nominal.attitude.z);
    printf("  (z component is sin(yaw/2), showing accumulated yaw rotation)\n");
  }
  printf("\n");

  // === Scenario: Coast phase (drag + GPS velocity can constrain attitude) ===
  printf("--- Scenario: Coast phase (drag + GPS improve attitude) ---\n");
  {
    // Simulate rocket coasting upward with some wobble
    // GPS says we're moving north at 100 m/s (velocity direction = north)
    GPSMeasurement gps_coast;
    gps_coast.position = Vec3(0, 0, 5000);  // 5km altitude
    gps_coast.velocity = Vec3(100, 0, -10);  // moving north, slight descent
    gps_coast.position_noise = 5.0f;
    gps_coast.velocity_noise = 0.5f;

    printf("GPS: velocity pointing north (100, 0, -10) m/s\n");
    eskfUpdateGPS(filter, gps_coast);
    printf("After GPS update: attitude refined via weathercock\n");
    printf("  (nose should point in direction of velocity)\n");

    // Now apply drag pseudo-measurement
    // Rocket is still accelerating downward due to gravity and drag
    DragPseudoMeasurement drag;
    drag.world_velocity = gps_coast.velocity;
    // Accelerometer reads combination of gravity and drag (in body frame)
    // If rocket is pointed north but tilted slightly, body-frame accel would be...
    // (this is simplified; real calc would account for attitude)
    drag.accel_body = Vec3(0, 0, 2.0f);  // mostly gravity at this point (2g residual)
    drag.confidence = 0.5f;  // modest confidence (low drag at apogee)

    printf("Accel-based drag inference: body accel = (0, 0, 2) m/s^2\n");
    printf("  (mostly residual gravity, drag not yet dominant)\n");
    eskfUpdateDrag(filter, drag);
    printf("After drag update: attitude uncertainty reduced\n\n");
  }

  // === Scenario: Drogue deploy (high drag, strong attitude constraint) ===
  printf("--- Scenario: Drogue deploy (high drag, strong attitude constraint) ---\n");
  {
    // High drag now
    DragPseudoMeasurement drag_drogue;
    drag_drogue.world_velocity = Vec3(0, 0, 50);  // falling at 50 m/s
    drag_drogue.accel_body = Vec3(0, 0, 8.0f);  // now mostly drag (8g)
    drag_drogue.confidence = 0.9f;  // high confidence in drag model here

    printf("High-drag descent: accel = (0, 0, 8) m/s^2 (mostly drag)\n");
    printf("Drag confidence: high (0.9)\n");
    eskfUpdateDrag(filter, drag_drogue);
    printf("After drag update: attitude strongly constrained\n\n");
  }

  printf("=== Final state ===\n");
  printf("Attitude: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]\n",
         filter.nominal.attitude.w, filter.nominal.attitude.x,
         filter.nominal.attitude.y, filter.nominal.attitude.z);
  printf("Position: [%.1f, %.1f, %.1f] m\n",
         filter.nominal.position.x, filter.nominal.position.y, filter.nominal.position.z);
  printf("Velocity: [%.1f, %.1f, %.1f] m/s\n",
         filter.nominal.velocity.x, filter.nominal.velocity.y, filter.nominal.velocity.z);
  printf("\nCovariance diagonal (uncertainty in each state):\n");
  for (int i = 0; i < 15; i++) {
    printf("  P[%d,0] = %.6f\n", i, filter.P.at(i, i));
  }

  printf("\n=== Key observations ===\n");
  printf("1. Nominal state is propagated forward with full nonlinear dynamics.\n");
  printf("2. Error state covariance (P) grows during predict, shrinks during update.\n");
  printf("3. GPS velocity measurement feeds into weathercock pseudo-measurement.\n");
  printf("4. Drag pseudo-measurement provides attitude constraint without knowing\n");
  printf("   absolute acceleration—only relative to velocity direction.\n");
  printf("5. All three attitude measurements (mag, GPS, drag) naturally integrate\n");
  printf("   in the single filter; they don't need special decomposition.\n");
  printf("6. Boost phase has no useful attitude updates (high-g accel, no mag/GPS);\n");
  printf("   coast phase gains strong constraints from both drag and aero assumption.\n");

  return 0;
}
