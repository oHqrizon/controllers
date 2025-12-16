#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <algorithm>

using namespace std;

// PID controller for cruise control of a car

class PIDController {
  private:
    // Gains
    const double kp;
    const double ki;
    const double kd;

    // Errors
    double error = 0;
    double prevError = 0;
    double totalError = 0;

  public:
    PIDController(double kp_, double ki_, double kd_) : kp(kp_), ki(ki_), kd(kd_) { 
      cout << "PID Initialized" << endl;
    }

    double update(double v_ref, double v_curr, const double dt){
        // Goal: Given v_ref, compute a u (throttle) to reach this v_ref

        prevError = error; // Kd
        error = v_ref - v_curr; // Kp 
        totalError += error; // Ki

        double proportional = kp * error;
        double integral = ki * totalError * dt; // Discrete-time approximation
        double derivative = kd * (error - prevError) / dt;

        double u = proportional + integral + derivative;

        // Anti wind-up (if saturated, don't include any more errors)
        if (u < 0.0 || u > 1.0){
          totalError-=error;
        }

        u = clamp(u, 0.0, 1.0);
        
        // PID control
        return u; 
    }

};

constexpr double dt = 0.01; // Loop runs at 100Hz
constexpr int steps = 4500;

void generateReference(vector<double> &v_ref){
  // Total of 45 seconds
  for (int i = 0; i < steps; ++i) {
      double t = i * dt;
      if (t < 5.0)
          v_ref[i] = 0.0;
      else if (t < 10.0)
          v_ref[i] = 20.0;
      else if (t < 25.0)
          v_ref[i] = 30.0;
      else if (t < 35.0)
          v_ref[i] = 25.0;
      else
          v_ref[i] = 15.0;
  }
}

double simulateControl(double throttle, double currSpeed){

  throttle = clamp(throttle, 0.0 , 1.0);

  const double mass = 1800;
  const double k = 10000;
  const double b = 120; // Some drag
  const double d = 160; // Some external force

  double accel = (k * throttle - b * currSpeed - d) / mass;
  double future_v = currSpeed + accel * dt;

  return future_v;

}

int main(){

  vector<double> v_ref(steps);
  generateReference(v_ref);

  PIDController pid = PIDController(0.5, 0.3, 0.1);

  double throttle = 0;
  double v_curr = 0;

  for (double velocity : v_ref){
    // Get time now
    auto loop_start = std::chrono::high_resolution_clock::now();

    cout << "Throttle: " << throttle << endl;
    cout << "Current Speed: " << v_curr << endl;
    cout << "Reference Speed: " << velocity << endl;

    throttle = pid.update(velocity, v_curr, dt);
    v_curr = simulateControl(throttle, v_curr);
    
    // Ensure 100Hz loop
    auto loop_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = loop_end - loop_start;
    
    double sleep_time = dt - elapsed.count();
    if(sleep_time > 0){
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
    }
  }

  return 0;
}
