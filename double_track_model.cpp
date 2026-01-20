#include "eufs_models/DoubleTrack.hpp"
#include <cmath>

namespace eufs {
namespace models {

DoubleTrack::DoubleTrack(const std::string& yaml_file) : VehicleModel(yaml_file) {}



void DoubleTrack::updateState(State& state, Input& input, double dt) {
  validateInput(input);

  // 1. Normal force distribution
  double Fz_total = computeNormalForce(state);
  double w_front = _param.kinematic.w_front;

  double Fz_front = w_front * Fz_total;
  double Fz_rear  = (1.0 - w_front) * Fz_total;

  double Fz_fl = 0.5 * Fz_front;
  double Fz_fr = 0.5 * Fz_front;
  double Fz_rl = 0.5 * Fz_rear;
  double Fz_rr = 0.5 * Fz_rear;


  // 2. Compute slip angles per tire
  double alpha_fl = computeSlipAngle(state, input, +1, +1);
  double alpha_fr = computeSlipAngle(state, input, +1, -1);
  double alpha_rl = computeSlipAngle(state, input, -1, +1);
  double alpha_rr = computeSlipAngle(state, input, -1, -1);

  // 3. Lateral forces per wheel
  double Fy_fl = computeLateralForce(Fz_fl, alpha_fl);
  double Fy_fr = computeLateralForce(Fz_fr, alpha_fr);
  double Fy_rl = computeLateralForce(Fz_rl, alpha_rl);
  double Fy_rr = computeLateralForce(Fz_rr, alpha_rr);

  // 4. Longitudinal force (distributed evenly to 4 wheels for now)
  double Fx_total = computeTotalFx(state, input);
  double Fx_fl = 0.25 * Fx_total;
  double Fx_fr = 0.25 * Fx_total;
  double Fx_rl = 0.25 * Fx_total;
  double Fx_rr = 0.25 * Fx_total;

  // 5. Compute full dynamics
  State x_dot = computeDynamics(state, Fx_fl, Fx_fr, Fx_rl, Fx_rr,
                                       Fy_fl, Fy_fr, Fy_rl, Fy_rr,
                                       input.delta);

  // 6. Integrate (Euler)
  state = state + x_dot * dt;

  // 7. Compute per-wheel velocities
  computeWheelVelocities(state, input);

  validateState(state);
}


// ------------------ DYNAMICS ------------------

State DoubleTrack::computeDynamics(const State& x,
                                   double Fx_fl, double Fx_fr, double Fx_rl, double Fx_rr,
                                   double Fy_fl, double Fy_fr, double Fy_rl, double Fy_rr,
                                   double delta) {
  const double m = _param.inertia.m;
  const double Iz = _param.inertia.I_z;
  const double l_F = _param.kinematic.l_F;
  const double l_R = _param.kinematic.l_R;
  const double w = _param.kinematic.axle_width;

  State x_dot{};

  double Fx_total = Fx_fl + Fx_fr + Fx_rl + Fx_rr;
  double Fy_total = Fy_fl + Fy_fr + Fy_rl + Fy_rr;

  // Moments: Mz = torque around z from lateral and longitudinal forces
  double Mz =
    (Fy_fl + Fy_fr) * l_F * std::cos(delta) - (Fy_rl + Fy_rr) * l_R +     // lateral lever
    (Fx_fr + Fx_rr - Fx_fl - Fx_rl) * (w / 2.0);                          // torque from longitudinal imbalance

  // Global motion derivatives
  x_dot.x    = std::cos(x.yaw) * x.v_x - std::sin(x.yaw) * x.v_y;
  x_dot.y    = std::sin(x.yaw) * x.v_x + std::cos(x.yaw) * x.v_y;
  x_dot.yaw  = x.r_z;

  // Body frame velocity dynamics
  x_dot.v_x  = x.r_z * x.v_y + Fx_total / m;
  x_dot.v_y  = -x.r_z * x.v_x + Fy_total / m;
  x_dot.r_z  = Mz / Iz;

  return x_dot;
}

// ------------------ FORCE MODELS ------------------

double DoubleTrack::computeTotalFx(const State& x, const Input& u) {
  double acc = (x.v_x <= 0.0 && u.acc < 0.0) ? 0.0 : u.acc;
  return acc * _param.inertia.m - computeDragForce(x);
}

double DoubleTrack::computeDragForce(const State& x) {
  return _param.aero.c_drag * x.v_x * x.v_x;
}

double DoubleTrack::computeDownforce(const State& x) {
  return _param.aero.c_down * x.v_x * x.v_x;
}

double DoubleTrack::computeNormalForce(const State& x) {
  return _param.inertia.g * _param.inertia.m + computeDownforce(x);
}

double DoubleTrack::computeLateralForce(double Fz, double slip_angle) {
  const double B = _param.tire.B;
  const double C = _param.tire.C;
  const double D = _param.tire.D;
  const double E = _param.tire.E;

  double mu_y = D * std::sin(C * std::atan(B * (1.0 - E) * slip_angle + E * std::atan(B * slip_angle)));
  return Fz * mu_y;
}

// ------------------ SLIP ANGLE ------------------

double DoubleTrack::computeSlipAngle(const State& x, const Input& u, int axle_sign, int side_sign) {
  double l = (axle_sign > 0) ? _param.kinematic.l_F : -_param.kinematic.l_R;
  double y_offset = side_sign * _param.kinematic.axle_width / 2.0;
  double delta = (axle_sign > 0) ? u.delta : 0.0; //if rear axle, it will always go straight

  double v_x_local = x.v_x - x.r_z * y_offset;
  double v_y_local = x.v_y + x.r_z * l;

  return std::atan2(v_y_local, v_x_local) - delta;
}

// ------------------ WHEEL VELOCITIES ------------------

void DoubleTrack::computeWheelVelocities(State& state, const Input& input) {
  const double l_F = _param.kinematic.l_F;
  const double l_R = _param.kinematic.l_R;
  const double w = _param.kinematic.axle_width;
  const double delta = input.delta;

  auto wheelVelocity = [&](double x_offset, double y_offset) {
    double v_x = state.v_x - state.r_z * y_offset;
    double v_y = state.v_y + state.r_z * x_offset;
    return std::make_pair(v_x, v_y);
  };

  auto v_fl = wheelVelocity(l_F, +w / 2);
  auto v_fr = wheelVelocity(l_F, -w / 2);
  auto v_rl = wheelVelocity(-l_R, +w / 2);
  auto v_rr = wheelVelocity(-l_R, -w / 2);

  state.v_fl = v_fl.first * std::cos(delta) + v_fl.second * std::sin(delta);
  state.v_fr = v_fr.first * std::cos(delta) + v_fr.second * std::sin(delta);
  state.v_rl = v_rl.first;
  state.v_rr = v_rr.first;
}

}  // namespace models
}  // namespace eufs
