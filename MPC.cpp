class MPC {
public:
    struct State {
        double y;    // lateral displacement [m]
        double v_y;  // lateral velocity [m/s]
        double psi;  // yaw angle [rad]
        double r;    // yaw rate [rad/s]
    };

    struct Control {
        double delta; // steering angle [rad]
    };

    struct Params {
        double m;    // mass [kg]
        double I_z;  // yaw inertia [kg·m^2]
        double l_f;  // distance CoG -> front axle [m]
        double l_r;  // distance CoG -> rear axle [m]
        double C_f;  // front cornering stiffness in negative [N/rad]
        double C_r;  // rear cornering stiffness in negative [N/rad]
    };

    static void buildLinearModel(const State& x, const Control& u, 
                                 Eigen::Matrix<double, 4, 4>& A,
                                 Eigen::Matrix<double, 4, 1>& B,
                                 Eigen::Matrix<double, 4, 1>& c,
                                 const Params& p, double v_x)
    {
        // --- Parse state and input ---
        double y     = x.y;
        double v_y   = x.v_y;
        double psi   = x.psi;
        double r     = x.r;
        double delta = u.delta;

        // --- Initialize matrices ---
        A.setZero();
        B.setZero();
        c.setZero();

        // --- A matrix ---
        A(0,0) = 0.0;
        A(0,1) = 1.0;
        A(0,2) = v_x;
        A(0,3) = 0.0;

        A(1,0) = 0.0;
        A(1,1) = (p.C_f + p.C_r) / (p.m * v_x);
        A(1,2) = 0.0;
        A(1,3) = (p.l_f * p.C_f - p.l_r * p.C_r) / (p.m * v_x) - v_x;

        A(2,0) = 0.0;
        A(2,1) = 0.0;
        A(2,2) = 0.0;
        A(2,3) = 1.0;

        A(3,0) = 0.0;
        A(3,1) = (p.l_f * p.C_f - p.l_r * p.C_r) / (p.I_z * v_x);
        A(3,2) = 0.0;
        A(3,3) = (p.l_f * p.l_f * p.C_f + p.l_r * p.l_r * p.C_r) / (p.I_z * v_x);


        // --- B matrix ---
        B(0,0) = 0.0;
        B(1,0) = -p.C_f / p.m;
        B(2,0) = 0.0;
        B(3,0) = -p.l_f * p.C_f / p.I_z;

        //xdot (DYNAMIC BICCYLE MODEL EQUATIONS BELOW)
        State f;

        // Slip angles
        double alpha_f = -u.delta + std::atan2((v_y + p.l_f * r), v_x);
        double alpha_r = std::atan2((v_y - p.l_r * r), v_x);

        // Tire forces
        double Fy_f = p.C_f * alpha_f;
        double Fy_r = p.C_r * alpha_r;

        // Yaw moment
        double Mz = p.l_f * Fy_f * std::cos(u.delta) - p.l_r * Fy_r;

        // Continuous dynamics (xdot)
        f.y   = v_y;
        f.v_y = (Fy_f * std::cos(u.delta) + Fy_r) / p.m - v_x * r;
        f.psi = r;
        f.r   = Mz / p.I_z;

        Eigen::Matrix<double, 4, 1> x_vec;
        x_vec << y, v_y, psi, r;

        Eigen::Matrix<double, 1, 1> u_vec;
        u_vec << delta;

        Eigen::Matrix<double, 4, 1> f_vec;
        f_vec << f.y, f.v_y, f.psi, f.r;

        c = f_vec - A * x_vec - B * u_vec;
    }

    static std::tuple<Eigen::VectorXd, std::vector<double>, std::vector<std::pair<double,double>>>
    buildReference(const spline_msgs::msg::ParametricSpline& spline, int N, int NX, double v_x){

        std::vector<double> x_params = spline.x_params;
        std::vector<double> y_params = spline.y_params;

        // === Fixed timestep for MPC ===
        double dt = 0.04;

        // === Reference containers ===
        Eigen::VectorXd x_ref_vec(NX * N);
        std::vector<double> delta_ref(N, 0.0);
        std::vector<std::pair<double, double>> path_points; //for viz
        path_points.reserve(N);

        // === Vehicle geometry ===
        const double Lf = 0.811;  // front axle to CoG [m]
        const double Lr = 0.719;  // rear axle to CoG [m]
        const double L  = Lf + Lr; // wheelbase

        double s = 0.0;

        for (int i = 0; i < N; ++i){
              // --- Compute local derivatives wrt s ---
              double dx_ds = 5*x_params[0]*pow(s,4) + 4*x_params[1]*pow(s,3) +
                            3*x_params[2]*pow(s,2) + 2*x_params[3]*s + x_params[4];
              double dy_ds = 5*y_params[0]*pow(s,4) + 4*y_params[1]*pow(s,3) +
                            3*y_params[2]*pow(s,2) + 2*y_params[3]*s + y_params[4];

              // --- Compute local arc-length scaling ---
              double deriv_mag = std::sqrt(dx_ds*dx_ds + dy_ds*dy_ds);

              double max_ds = 0.04; //max step along spline to avoid overshoot
              double ds_arc = (v_x * dt) / deriv_mag;
              ds_arc = std::clamp(ds_arc, 0.0, max_ds);

              //Distance of x and y params from car
              double x_path = x_params[0] * pow(s, 5) + x_params[1] * pow(s, 4) +
                        x_params[2] * pow(s, 3) + x_params[3] * pow(s, 2) +
                        x_params[4] * s + x_params[5];

              double y_path = y_params[0] * pow(s, 5) + y_params[1] * pow(s, 4) +
                        y_params[2] * pow(s, 3) + y_params[3] * pow(s, 2) +
                        y_params[4] * s + y_params[5];

              path_points.emplace_back(x_path, y_path);

              double ddx_ds = 20*x_params[0]*pow(s,3) + 12*x_params[1]*pow(s,2) + 6*x_params[2]*s + 2*x_params[3];
              double ddy_ds = 20*y_params[0]*pow(s,3) + 12*y_params[1]*pow(s,2) + 6*y_params[2]*s + 2*y_params[3];


              // === Compute path heading and curvature ===
              double psi_desired = atan2(dy_ds, dx_ds);
              
              double denom = std::pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);
              double kappa = (denom > 1e-10) ? (dx_ds * ddy_ds - dy_ds * ddx_ds) / denom : 0;
              //kappa = std::clamp(kappa, -0.2, 0.2);

              double max_kappa = tan(0.7) / L;  //max angle of 0.34rads
              kappa = std::clamp(kappa, -max_kappa, max_kappa);


              // === Reference steering from curvature ===
              delta_ref[i] = std::atan(L * kappa);

              // === Reference yaw rate ===
              double r_ref = v_x * kappa;
              double max_r = 5.0;  // rad/s absolute safety limit
              r_ref = std::clamp(r_ref, -max_r, max_r);

              // === Reference lateral velocity ===
              double v_y_ref = 0;

              // === Pack state reference vector [y, v_y, psi, r] ===
              x_ref_vec.segment(i * NX, NX) << y_path, v_y_ref, psi_desired, r_ref;

              // --- Advance along spline ---
              s += ds_arc;
              s = std::clamp(s, 0.0, 1.0);
          }

          return std::make_tuple(x_ref_vec, delta_ref, path_points);
      }
};
