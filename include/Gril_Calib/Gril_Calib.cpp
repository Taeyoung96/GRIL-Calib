#include "Gril_Calib.h"

/*
Description: Gril-Calib (Heavily adapted from LI-Init by Fangcheng Zhu)
Modifier : Taeyoung Kim (https://github.com/Taeyoung96)
*/

Gril_Calib::Gril_Calib()
        : time_delay_IMU_wtr_Lidar(0.0), time_lag_1(0.0), time_lag_2(0.0), lag_IMU_wtr_Lidar(0) {
    fout_LiDAR_meas.open(FILE_DIR("LiDAR_meas.txt"), ios::out);
    fout_IMU_meas.open(FILE_DIR("IMU_meas.txt"), ios::out);
    fout_before_filt_IMU.open(FILE_DIR("IMU_before_filter.txt"), ios::out);
    fout_before_filt_Lidar.open(FILE_DIR("Lidar_before_filter.txt"), ios::out);
    fout_acc_cost.open(FILE_DIR("acc_cost.txt"), ios::out);
    fout_after_rot.open(FILE_DIR("Lidar_omg_after_rot.txt"), ios::out);

    fout_LiDAR_ang_vel.open(FILE_DIR("Lidar_ang_vel.txt"), ios::out);
    fout_IMU_ang_vel.open(FILE_DIR("IMU_ang_vel.txt"), ios::out);
    fout_Jacob_trans.open(FILE_DIR("Jacob_trans.txt"), ios::out);

    fout_LiDAR_meas_after.open(FILE_DIR("LiDAR_meas_after.txt"), ios::out);

   
    data_accum_length = 300;

    trans_IL_x = 0.0;
    trans_IL_y = 0.0;
    trans_IL_z = 0.0;
    bound_th = 0.1; 
    set_boundary = false;

    Rot_Grav_wrt_Init_Lidar = Eye3d;
    Trans_Lidar_wrt_IMU = Zero3d;
    Rot_Lidar_wrt_IMU = Eye3d;
    gyro_bias = Zero3d;
    acc_bias = Zero3d;
}

Gril_Calib::~Gril_Calib() = default;

void Gril_Calib::set_IMU_state(const deque<CalibState> &IMU_states) {
    IMU_state_group.assign(IMU_states.begin(), IMU_states.end() - 1);
}

void Gril_Calib::set_Lidar_state(const deque<CalibState> &Lidar_states) {
    Lidar_state_group.assign(Lidar_states.begin(), Lidar_states.end() - 1);
}

void Gril_Calib::set_states_2nd_filter(const deque<CalibState> &IMU_states, const deque<CalibState> &Lidar_states) {
    for (int i = 0; i < IMU_state_group.size(); i++) {
        IMU_state_group[i].ang_acc = IMU_states[i].ang_acc;
        Lidar_state_group[i].ang_acc = Lidar_states[i].ang_acc;
        Lidar_state_group[i].linear_acc = Lidar_states[i].linear_acc;
    }
}

void Gril_Calib::fout_before_filter() {
    for (auto it_IMU = IMU_state_group.begin(); it_IMU != IMU_state_group.end() - 1; it_IMU++) {
        fout_before_filt_IMU << setprecision(15) << it_IMU->ang_vel.transpose() << " " << it_IMU->ang_vel.norm() << " "
                             << it_IMU->linear_acc.transpose() << " " << it_IMU->timeStamp << endl;
    }
    for (auto it = Lidar_state_group.begin(); it != Lidar_state_group.end() - 1; it++) {
        fout_before_filt_Lidar << setprecision(15) << it->ang_vel.transpose() << " " << it->ang_vel.norm() << " "
                               << it->timeStamp << endl;
    }
}

void Gril_Calib::fout_check_lidar() {
    auto it_Lidar_state = Lidar_state_group.begin() + 1;
    for (; it_Lidar_state != Lidar_state_group.end() - 2; it_Lidar_state++) {
        fout_LiDAR_meas_after << setprecision(12) << it_Lidar_state->ang_vel.transpose() << " "
                        << it_Lidar_state->ang_vel.norm()
                        << " " <<
                        it_Lidar_state->linear_acc.transpose() << " "
                        << it_Lidar_state->ang_acc.transpose()
                        << " " << it_Lidar_state->timeStamp << endl;
    }
}


void Gril_Calib::push_ALL_IMU_CalibState(const sensor_msgs::Imu::ConstPtr &msg, const double &mean_acc_norm) {
    CalibState IMUstate;
    double invert = -1.0;
    IMUstate.ang_vel = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    IMUstate.linear_acc =
            V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) / mean_acc_norm *
            G_m_s2;
    
    IMUstate.timeStamp = msg->header.stamp.toSec();
    IMU_state_group_ALL.push_back(IMUstate);
}

void Gril_Calib::push_IMU_CalibState(const V3D &omg, const V3D &acc, const double &timestamp) {
    CalibState IMUstate;
    IMUstate.ang_vel = omg;
    IMUstate.linear_acc = acc;
    IMUstate.timeStamp = timestamp;
    IMU_state_group.push_back(IMUstate);
}

void Gril_Calib::push_Lidar_CalibState(const M3D &rot, const V3D &pos, const V3D &omg, const V3D &linear_vel, const double &timestamp) {
    CalibState Lidarstate;
    Lidarstate.rot_end = rot;
    Lidarstate.pos_end = pos;
    Lidarstate.ang_vel = omg;
    Lidarstate.linear_vel = linear_vel;
    Lidarstate.timeStamp = timestamp;
    Lidar_state_group.push_back(Lidarstate);
}

void Gril_Calib::push_Plane_Constraint(const Eigen::Quaterniond &q_lidar, const Eigen::Quaterniond &q_imu, const V3D &normal_lidar,
                                                                                                             const double &distance_lidar) {
    Lidar_wrt_ground_group.push_back(q_lidar);
    IMU_wrt_ground_group.push_back(q_imu);
    normal_vector_wrt_lidar_group.push_back(normal_lidar);
    distance_Lidar_wrt_ground_group.push_back(distance_lidar);
}

void Gril_Calib::downsample_interpolate_IMU(const double &move_start_time) {

    while (IMU_state_group_ALL.front().timeStamp < move_start_time - 3.0)
        IMU_state_group_ALL.pop_front();
    while (Lidar_state_group.front().timeStamp < move_start_time - 3.0)
        Lidar_state_group.pop_front();

    // Original IMU measurements
    deque<CalibState> IMU_states_all_origin;
    IMU_states_all_origin.assign(IMU_state_group_ALL.begin(), IMU_state_group_ALL.end() - 1);

    // Mean filter to attenuate noise
    int mean_filt_size = 3;
    for (int i = mean_filt_size; i < IMU_state_group_ALL.size() - mean_filt_size; i++) {
        V3D acc_real = Zero3d;
        for (int k = -mean_filt_size; k < mean_filt_size + 1; k++)
            acc_real += (IMU_states_all_origin[i + k].linear_acc - acc_real) / (k + mean_filt_size + 1);
        IMU_state_group_ALL[i].linear_acc = acc_real;
    }

    // Down-sample and interpolation，Fig.4 in the paper
    for (int i = 0; i < Lidar_state_group.size(); i++) {
        for (int j = 1; j < IMU_state_group_ALL.size(); j++) {
            if (IMU_state_group_ALL[j - 1].timeStamp <= Lidar_state_group[i].timeStamp
                && IMU_state_group_ALL[j].timeStamp > Lidar_state_group[i].timeStamp) {
                CalibState IMU_state_interpolation;
                double delta_t = IMU_state_group_ALL[j].timeStamp - IMU_state_group_ALL[j - 1].timeStamp;
                double delta_t_right = IMU_state_group_ALL[j].timeStamp - Lidar_state_group[i].timeStamp;
                double s = delta_t_right / delta_t;

                IMU_state_interpolation.ang_vel = s * IMU_state_group_ALL[j - 1].ang_vel +
                                                  (1 - s) * IMU_state_group_ALL[j].ang_vel;

                IMU_state_interpolation.linear_acc = s * IMU_state_group_ALL[j - 1].linear_acc +
                                                     (1 - s) * IMU_state_group_ALL[j].linear_acc;
                push_IMU_CalibState(IMU_state_interpolation.ang_vel, IMU_state_interpolation.linear_acc,
                                    Lidar_state_group[i].timeStamp);
                break;
            }
        }
    }

}

// Calculates the angular acceleration of the IMU, LiDAR, and linear acceleration.
void Gril_Calib::central_diff() {
    auto it_IMU_state = IMU_state_group.begin() + 1;
    for (; it_IMU_state != IMU_state_group.end() - 2; it_IMU_state++) {
        auto last_imu = it_IMU_state - 1;
        auto next_imu = it_IMU_state + 1;
        double dt_imu = next_imu->timeStamp - last_imu->timeStamp;
        it_IMU_state->ang_acc =
                (next_imu->ang_vel - last_imu->ang_vel) / dt_imu;
        fout_IMU_meas << setprecision(12) << it_IMU_state->ang_vel.transpose() << " " << it_IMU_state->ang_vel.norm()
                      << " "
                      <<
                      it_IMU_state->linear_acc.transpose() << " " << it_IMU_state->ang_acc.transpose() << " "
                      << it_IMU_state->timeStamp << endl;
    }

    auto it_Lidar_state = Lidar_state_group.begin() + 1;
    for (; it_Lidar_state != Lidar_state_group.end() - 2; it_Lidar_state++) {
        auto last_lidar = it_Lidar_state - 1;
        auto next_lidar = it_Lidar_state + 1;
        double dt_lidar = next_lidar->timeStamp - last_lidar->timeStamp;
        it_Lidar_state->ang_acc =
                (next_lidar->ang_vel - last_lidar->ang_vel) / dt_lidar;
        it_Lidar_state->linear_acc =
                (next_lidar->linear_vel - last_lidar->linear_vel) / dt_lidar;

        fout_LiDAR_meas << setprecision(12) << it_Lidar_state->ang_vel.transpose() << " "
                        << it_Lidar_state->ang_vel.norm()
                        << " " <<
                        (it_Lidar_state->linear_acc - STD_GRAV).transpose() << " "
                        << it_Lidar_state->ang_acc.transpose()
                        << " " << it_Lidar_state->timeStamp << endl;
    }
}

// Temporal calibration by Cross-Correlation : calculate time_lag_1
void Gril_Calib::xcorr_temporal_init(const double &odom_freq) {
    int N = IMU_state_group.size();
    //Calculate mean value of IMU and LiDAR angular velocity
    double mean_IMU_ang_vel = 0, mean_LiDAR_ang_vel = 0;
    for (int i = 0; i < N; i++) {
        mean_IMU_ang_vel += (IMU_state_group[i].ang_vel.norm() - mean_IMU_ang_vel) / (i + 1);
        mean_LiDAR_ang_vel += (Lidar_state_group[i].ang_vel.norm() - mean_LiDAR_ang_vel) / (i + 1);
    }

    //Calculate zero-centered cross correlation
    double max_corr = -DBL_MAX;
    for (int lag = -N + 1; lag < N; lag++) {
        double corr = 0;
        int cnt = 0;
        for (int i = 0; i < N; i++) {
            int j = i + lag;
            if (j < 0 || j > N - 1)
                continue;
            else {
                cnt++;
                corr += (IMU_state_group[i].ang_vel.norm() - mean_IMU_ang_vel) *
                        (Lidar_state_group[j].ang_vel.norm() - mean_LiDAR_ang_vel);  // Zero-centered cross correlation
            }
        }

        if (corr > max_corr) {
            max_corr = corr;
            lag_IMU_wtr_Lidar = -lag;
        }
    }

    time_lag_1 = lag_IMU_wtr_Lidar / odom_freq;
    cout << "Max Cross-correlation: IMU lag wtr Lidar : " << -lag_IMU_wtr_Lidar << endl;
    cout << "Time lag 1: IMU lag wtr Lidar : " << time_lag_1 << endl;
}

void Gril_Calib::IMU_time_compensate(const double &lag_time, const bool &is_discard) {
    if (is_discard) {
        // Discard first 10 Lidar estimations and corresponding IMU measurements due to long time interval
        int i = 0;
        while (i < 10) {
            Lidar_state_group.pop_front();
            IMU_state_group.pop_front();
            i++;
        }
    }

    auto it_IMU_state = IMU_state_group.begin();
    for (; it_IMU_state != IMU_state_group.end() - 1; it_IMU_state++) {
        it_IMU_state->timeStamp = it_IMU_state->timeStamp - lag_time;
    }

    while (Lidar_state_group.front().timeStamp < IMU_state_group.front().timeStamp)
        Lidar_state_group.pop_front();  

    while (Lidar_state_group.front().timeStamp > IMU_state_group[1].timeStamp)
        IMU_state_group.pop_front();    

    // align the size of two sequences
    while (IMU_state_group.size() > Lidar_state_group.size())
        IMU_state_group.pop_back();
    while (IMU_state_group.size() < Lidar_state_group.size())
        Lidar_state_group.pop_back();
}


void Gril_Calib::cut_sequence_tail() {
    for (int i = 0; i < 20; ++i) {
        Lidar_state_group.pop_back();
        IMU_state_group.pop_back();
    }
    while (Lidar_state_group.front().timeStamp < IMU_state_group.front().timeStamp)
        Lidar_state_group.pop_front();
    while (Lidar_state_group.front().timeStamp > IMU_state_group[1].timeStamp)
        IMU_state_group.pop_front();

    //Align the size of two sequences
    while (IMU_state_group.size() > Lidar_state_group.size())
        IMU_state_group.pop_back();
    while (IMU_state_group.size() < Lidar_state_group.size())
        Lidar_state_group.pop_back();
}

void Gril_Calib::acc_interpolate() {
    //Interpolation to get acc_I(t_L)
    for (int i = 1; i < Lidar_state_group.size() - 1; i++) {
        double deltaT = Lidar_state_group[i].timeStamp - IMU_state_group[i].timeStamp;
        if (deltaT > 0) {
            double DeltaT = IMU_state_group[i + 1].timeStamp - IMU_state_group[i].timeStamp;
            double s = deltaT / DeltaT;
            IMU_state_group[i].linear_acc = s * IMU_state_group[i + 1].linear_acc +
                                            (1 - s) * IMU_state_group[i].linear_acc;
            IMU_state_group[i].timeStamp += deltaT;
        } else {
            double DeltaT = IMU_state_group[i].timeStamp - IMU_state_group[i - 1].timeStamp;
            double s = -deltaT / DeltaT;
            IMU_state_group[i].linear_acc = s * IMU_state_group[i - 1].linear_acc +
                                            (1 - s) * IMU_state_group[i].linear_acc;
            IMU_state_group[i].timeStamp += deltaT;
        }
    }
}

// Butterworth filter (Low-pass filter)
void Gril_Calib::Butter_filt(const deque<CalibState> &signal_in, deque<CalibState> &signal_out) {
    Gril_Calib::Butterworth butter;
    butter.extend_num = 10 * (butter.Coeff_size - 1);
    auto it_front = signal_in.begin() + butter.extend_num;
    auto it_back = signal_in.end() - 1 - butter.extend_num;

    deque<CalibState> extend_front;
    deque<CalibState> extend_back;

    for (int idx = 0; idx < butter.extend_num; idx++) {
        extend_front.push_back(*it_front);
        extend_back.push_front(*it_back);
        it_front--;
        it_back++;
    }

    deque<CalibState> sig_extended(signal_in);
    while (!extend_front.empty()) {
        sig_extended.push_front(extend_front.back());
        extend_front.pop_back();
    }
    while (!extend_back.empty()) {
        sig_extended.push_back(extend_back.front());
        extend_back.pop_front();
    }

    deque<CalibState> sig_out(sig_extended);
    // One-direction Butterworth filter Starts (all states)
    for (int i = butter.Coeff_size; i < sig_extended.size() - butter.extend_num; i++) {
        CalibState temp_state;
        for (int j = 0; j < butter.Coeff_size; j++) {
            auto it_sig_ext = *(sig_extended.begin() + i - j);
            temp_state += it_sig_ext * butter.Coeff_b[j];
        }
        for (int jj = 1; jj < butter.Coeff_size; jj++) {
            auto it_sig_out = *(sig_out.begin() + i - jj);
            temp_state -= it_sig_out * butter.Coeff_a[jj];
        }
        sig_out[i] = temp_state;
    }

    for (auto it = sig_out.begin() + butter.extend_num; it != sig_out.end() - butter.extend_num; it++) {
        signal_out.push_back(*it);
    }
}

// zero phase low-pass filter //
void Gril_Calib::zero_phase_filt(const deque<CalibState> &signal_in, deque<CalibState> &signal_out) {
    deque<CalibState> sig_out1;
    Butter_filt(signal_in, sig_out1);   // signal_in에 대해 Butterworth filter를 적용한 결과를 sig_out1에 저장

    deque<CalibState> sig_rev(sig_out1);
    reverse(sig_rev.begin(), sig_rev.end()); //Reverse the elements

    Butter_filt(sig_rev, signal_out);
    reverse(signal_out.begin(), signal_out.end()); //Reverse the elements
}


// To obtain a rough initial value of rotation matrix //
void Gril_Calib::solve_Rotation_only() {
    double R_LI_quat[4];

    R_LI_quat[0] = 1;
    R_LI_quat[1] = 0;
    R_LI_quat[2] = 0;
    R_LI_quat[3] = 0;

    ceres::LocalParameterization *quatParam = new ceres::QuaternionParameterization();
    ceres::Problem problem_rot;
    problem_rot.AddParameterBlock(R_LI_quat, 4, quatParam);

    for (int i = 0; i < IMU_state_group.size(); i++) {
        M3D Lidar_angvel_skew;
        Lidar_angvel_skew << SKEW_SYM_MATRX(Lidar_state_group[i].ang_vel);
        problem_rot.AddResidualBlock(Angular_Vel_Cost_only_Rot::Create(IMU_state_group[i].ang_vel,
                                                                       Lidar_state_group[i].ang_vel),
                                     nullptr,
                                     R_LI_quat);
    }

    ceres::Solver::Options options_quat;
    ceres::Solver::Summary summary_quat;
    ceres::Solve(options_quat, &problem_rot, &summary_quat);
    Eigen::Quaterniond q_LI(R_LI_quat[0], R_LI_quat[1], R_LI_quat[2], R_LI_quat[3]);
    Rot_Lidar_wrt_IMU = q_LI.matrix();  // LiDAR angulr velocity in IMU frame (from LiDAR to IMU)

}



// Proposed algorithm (Rotation + Translation)
void Gril_Calib::solve_Rot_Trans_calib(double &timediff_imu_wrt_lidar, const double &imu_sensor_height) {

    M3D R_IL_init = Rot_Lidar_wrt_IMU.transpose(); // Initial value of Rotation of IL (from IMU frame to LiDAR frame)
    Eigen::Quaterniond quat(R_IL_init);
    double R_IL_quat[4];
    R_IL_quat[0] = quat.w();
    R_IL_quat[1] = quat.x();
    R_IL_quat[2] = quat.y();
    R_IL_quat[3] = quat.z();

    double Trans_IL[3];             // Initial value of Translation of IL (IMU with respect to Lidar) - ceres solver input
    Trans_IL[0] = trans_IL_x;
    Trans_IL[1] = trans_IL_y;
    Trans_IL[2] = trans_IL_z;

    double bias_g[3];               // Initial value of gyro bias
    bias_g[0] = 0;
    bias_g[1] = 0;
    bias_g[2] = 0;

    double bias_aL[3];              // Initial value of acc bias
    bias_aL[0] = 0;
    bias_aL[1] = 0;
    bias_aL[2] = 0;

    double time_lag2 = 0;           // Second time lag (IMU wtr Lidar)

    // Define problem
    ceres::Problem problem_Ex_calib;
    ceres::LocalParameterization *quatParam = new ceres::QuaternionParameterization();
    
    // Define Loss function
    ceres::LossFunction *loss_function_angular = new ceres::CauchyLoss(0.5);
    ceres::ScaledLoss *loss_function_angular_scaled = new ceres::ScaledLoss(loss_function_angular, 0.5, ceres::TAKE_OWNERSHIP);

    ceres::LossFunction *loss_function_acc = new ceres::CauchyLoss(0.5);
    ceres::ScaledLoss *scaled_loss_acc = new ceres::ScaledLoss(loss_function_acc, 0.2, ceres::TAKE_OWNERSHIP);

    ceres::LossFunction *loss_function_plain_motion = new ceres::HuberLoss(0.5);
    ceres::ScaledLoss *loss_function_plain_motion_scaled = new ceres::ScaledLoss(loss_function_plain_motion, 0.3, ceres::TAKE_OWNERSHIP);

    // Add Parameter Block
    problem_Ex_calib.AddParameterBlock(R_IL_quat, 4, quatParam);
    problem_Ex_calib.AddParameterBlock(Trans_IL, 3);
    problem_Ex_calib.AddParameterBlock(bias_g, 3);
    problem_Ex_calib.AddParameterBlock(bias_aL, 3);

    //Jacobian of acc_bias, gravity, Translation
    int Jaco_size = 3 * Lidar_state_group.size();
    MatrixXd Jacobian(Jaco_size, 9);
    Jacobian.setZero();

    // Jacobian of Translation
    MatrixXd Jaco_Trans(Jaco_size, 3);
    Jaco_Trans.setZero();

    // Add Residual Block
    for (int i = 0; i < IMU_state_group.size(); i++) {
        double deltaT = Lidar_state_group[i].timeStamp - IMU_state_group[i].timeStamp;
    
        problem_Ex_calib.AddResidualBlock(Ground_Plane_Cost_IL::Create(Lidar_wrt_ground_group[i],
                                                                      IMU_wrt_ground_group[i],
                                                                      distance_Lidar_wrt_ground_group[i],
                                                                      imu_sensor_height),
                                                            loss_function_plain_motion_scaled,
                                                        R_IL_quat,
                                                        Trans_IL);

        problem_Ex_calib.AddResidualBlock(Angular_Vel_IL_Cost::Create(IMU_state_group[i].ang_vel,
                                                                  IMU_state_group[i].ang_acc,
                                                                  Lidar_state_group[i].ang_vel,
                                                                  deltaT),
                                            loss_function_angular_scaled,
                                         R_IL_quat,
                                         bias_g,
                                         &time_lag2);

        problem_Ex_calib.AddResidualBlock(Linear_acc_Rot_Cost_without_Gravity::Create(Lidar_state_group[i],
                                                                                      IMU_state_group[i].linear_acc,
                                                                                      Lidar_wrt_ground_group[i]),
                                                                                scaled_loss_acc,
                                                                            R_IL_quat,
                                                                            bias_aL,
                                                                            Trans_IL);

        Jacobian.block<3, 3>(3 * i, 0) = -Lidar_state_group[i].rot_end;
        Jacobian.block<3, 3>(3 * i, 3) << SKEW_SYM_MATRX(STD_GRAV);
        M3D omg_skew, angacc_skew;
        omg_skew << SKEW_SYM_MATRX(Lidar_state_group[i].ang_vel);
        angacc_skew << SKEW_SYM_MATRX(Lidar_state_group[i].ang_acc);
        M3D Jaco_trans_i = -omg_skew * omg_skew - angacc_skew;
        Jaco_Trans.block<3, 3>(3 * i, 0) = Jaco_trans_i;
        Jacobian.block<3, 3>(3 * i, 6) = Jaco_trans_i;
    }

    // Set boundary
    for (int index = 0; index < 3; ++index) {
        problem_Ex_calib.SetParameterUpperBound(bias_aL, index, 0.01);
        problem_Ex_calib.SetParameterLowerBound(bias_aL, index, -0.01);

        problem_Ex_calib.SetParameterUpperBound(bias_g, index, 0.01);
        problem_Ex_calib.SetParameterLowerBound(bias_g, index, -0.01);
    }

    if(set_boundary) {
        for (int index = 0; index < 3; ++index) {
            problem_Ex_calib.SetParameterUpperBound(Trans_IL, index, Trans_IL[index] + bound_th);
            problem_Ex_calib.SetParameterLowerBound(Trans_IL, index, Trans_IL[index] - bound_th);
        }
    }
    
    // Solver options
    ceres::Solver::Options options_Ex_calib;
    options_Ex_calib.num_threads = 1;
    options_Ex_calib.use_explicit_schur_complement = true;
    options_Ex_calib.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options_Ex_calib.preconditioner_type = ceres::SCHUR_JACOBI;
    options_Ex_calib.minimizer_progress_to_stdout = false;

    // Solve
    ceres::Solver::Summary summary_Ex_calib;    
    ceres::Solve(options_Ex_calib, &problem_Ex_calib, &summary_Ex_calib);

    // std::cout << summary_Ex_calib.FullReport() << "\n";
    
    //** Update the result **//

    // Rotation matrix
    Eigen::Quaterniond q_IL_final(R_IL_quat[0], R_IL_quat[1], R_IL_quat[2], R_IL_quat[3]);  // quaternion from IMU frame to Lidar frame
    Rot_Lidar_wrt_IMU = q_IL_final.matrix().transpose();
    V3D euler_angle = RotMtoEuler(Rot_Lidar_wrt_IMU);

    // Translation vector
    V3D Trans_IL_vec(Trans_IL[0], Trans_IL[1], Trans_IL[2]);
    Trans_Lidar_wrt_IMU = -1.0 * Rot_Lidar_wrt_IMU * Trans_IL_vec;

    // gravity vector - not used
    M3D R_WLO = Lidar_wrt_ground_group[0].matrix();
    Grav_L0 = R_WLO * STD_GRAV;   // gravity in first lidar frame

    // bias acc
    V3D bias_a_Lidar(bias_aL[0], bias_aL[1], bias_aL[2]);
    acc_bias = Rot_Lidar_wrt_IMU * bias_a_Lidar;

    // bias gyro
    gyro_bias = V3D(bias_g[0], bias_g[1], bias_g[2]);

    // time offset
    time_lag_2 = time_lag2;
    time_delay_IMU_wtr_Lidar = time_lag_1 + time_lag_2;

    time_offset_result = time_delay_IMU_wtr_Lidar;
   
    //The second temporal compensation
    IMU_time_compensate(get_lag_time_2(), false);

    // For debug
    for (int i = 0; i < IMU_state_group.size(); i++) {
        M3D R_GL = Lidar_wrt_ground_group[i].matrix();
        V3D Grav_L = R_GL * STD_GRAV;

        V3D acc_I = Lidar_state_group[i].rot_end * Rot_Lidar_wrt_IMU.transpose() * IMU_state_group[i].linear_acc -
                    Lidar_state_group[i].rot_end * bias_a_Lidar;
        V3D acc_L = Lidar_state_group[i].linear_acc +
                    Lidar_state_group[i].rot_end * Jaco_Trans.block<3, 3>(3 * i, 0) * Trans_IL_vec - Grav_L;
        fout_acc_cost << setprecision(10) << acc_I.transpose() << " " << acc_L.transpose() << " "
                      << IMU_state_group[i].timeStamp << " " << Lidar_state_group[i].timeStamp << endl;
    }
}

void Gril_Calib::normalize_acc(deque<CalibState> &signal_in) {
    V3D mean_acc(0, 0, 0);

    for (int i = 1; i < 10; i++) {
        mean_acc += (signal_in[i].linear_acc - mean_acc) / i;
    }

    for (int i = 0; i < signal_in.size(); i++) {
        signal_in[i].linear_acc = signal_in[i].linear_acc / mean_acc.norm() * G_m_s2;
    }
}

// Align the size of various states
void Gril_Calib::align_Group(const deque<CalibState> &IMU_states, deque<Eigen::Quaterniond> &Lidar_wrt_ground_states, 
                     deque<Eigen::Quaterniond> &IMU_wrt_ground_states, deque<V3D> &normal_vector_wrt_lidar_group, deque<double> &distance_Lidar_ground_states) {

    // Align the size of two sequences
    while(IMU_states.size() < Lidar_wrt_ground_states.size()) {
        Lidar_wrt_ground_states.pop_back();
        IMU_wrt_ground_states.pop_back();
        normal_vector_wrt_lidar_group.pop_back();
        distance_Lidar_ground_states.pop_back();
    }

}

bool Gril_Calib::data_sufficiency_assess(MatrixXd &Jacobian_rot, int &frame_num, V3D &lidar_omg, int &orig_odom_freq,
                                      int &cut_frame_num, QD &lidar_q, QD &imu_q, double &lidar_estimate_height) {
    //Calculation of Rotation Jacobian
    M3D lidar_omg_skew;
    lidar_omg_skew << SKEW_SYM_MATRX(lidar_omg);
    Jacobian_rot.block<3, 3>(3 * frame_num, 0) = lidar_omg_skew;
    bool data_sufficient = false;

    //Give a Data Appraisal every second
    if (frame_num % orig_odom_freq * cut_frame_num == 0) {
        M3D Hessian_rot = Jacobian_rot.transpose() * Jacobian_rot;
        EigenSolver<M3D> es(Hessian_rot);
        V3D EigenValue = es.eigenvalues().real();
        M3D EigenVec_mat = es.eigenvectors().real();

        M3D EigenMatCwise = EigenVec_mat.cwiseProduct(EigenVec_mat);
        std::vector<double> EigenMat_1_col{EigenMatCwise(0, 0), EigenMatCwise(1, 0), EigenMatCwise(2, 0)};
        std::vector<double> EigenMat_2_col{EigenMatCwise(0, 1), EigenMatCwise(1, 1), EigenMatCwise(2, 1)};
        std::vector<double> EigenMat_3_col{EigenMatCwise(0, 2), EigenMatCwise(1, 2), EigenMatCwise(2, 2)};

        // Find the maximum value of each column
        int maxPos[3] = {0};
        maxPos[0] = max_element(EigenMat_1_col.begin(), EigenMat_1_col.end()) - EigenMat_1_col.begin();
        maxPos[1] = max_element(EigenMat_2_col.begin(), EigenMat_2_col.end()) - EigenMat_2_col.begin();
        maxPos[2] = max_element(EigenMat_3_col.begin(), EigenMat_3_col.end()) - EigenMat_3_col.begin();

        V3D Scaled_Eigen = EigenValue / data_accum_length;   // the larger data_accum_length is, the more data is needed
        V3D Rot_percent(Scaled_Eigen[1] * Scaled_Eigen[2],
                        Scaled_Eigen[0] * Scaled_Eigen[2],
                        Scaled_Eigen[0] * Scaled_Eigen[1]);

        int axis[3];
        axis[2] = max_element(maxPos, maxPos + 3) - maxPos;
        axis[0] = min_element(maxPos, maxPos + 3) - maxPos;
        axis[1] = 3 - (axis[0] + axis[2]);

        double percentage_x = Rot_percent[axis[0]] < x_accumulate ? Rot_percent[axis[0]] : 1;
        double percentage_y = Rot_percent[axis[1]] < y_accumulate ? Rot_percent[axis[1]] : 1;
        double percentage_z = Rot_percent[axis[2]] < z_accumulate ? Rot_percent[axis[2]] : 1;
    
        clear(); //clear the screen
        printf("\033[3A\r");

        printProgress(percentage_x, 88);
        printProgress(percentage_y, 89);
        printProgress(percentage_z, 90);

        if(verbose){
            M3D R_GL = lidar_q.toRotationMatrix();
            M3D R_GI = imu_q.toRotationMatrix();

            printf(BOLDREDPURPLE "[Rotation matrix Ground to LiDAR (euler)] " RESET);
            cout << setprecision(4) << RotMtoEuler(R_GL).transpose() * 57.3 << " deg" << '\n';
                  
            printf(BOLDREDPURPLE "[Rotation matrix Ground to IMU (euler)] " RESET);
            cout << setprecision(4) << RotMtoEuler(R_GI).transpose() * 57.3 << " deg" << '\n';

            printf(BOLDREDPURPLE "[Estimated LiDAR sensor height] : " RESET);
            cout << setprecision(4) << lidar_estimate_height << " m" << '\n';
        }
        

        fflush(stdout);
        if (Rot_percent[axis[0]] > x_accumulate && Rot_percent[axis[1]] > y_accumulate && Rot_percent[axis[2]] > z_accumulate) {
            printf(BOLDCYAN "[calibration] Data accumulation finished, Lidar IMU calibration begins.\n\n" RESET);
            printf(BOLDBLUE"============================================================ \n\n" RESET);
            data_sufficient = true;
        }
    }

    if (data_sufficient)
        return true;
    else
        return false;
}


void Gril_Calib::printProgress(double percentage, int axis_ascii) {
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf(BOLDCYAN "[Data accumulation] ");
    if (percentage < 1) {
        printf(BOLDYELLOW "Rotation around Lidar %c Axis: ", char(axis_ascii));
        printf(YELLOW "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        cout << RESET;
    } else {
        printf(BOLDGREEN "Rotation around Lidar %c Axis complete! ", char(axis_ascii));
        cout << RESET << "\n";
    }
}

void Gril_Calib::clear() {
    // CSI[2J clears screen, CSI[H moves the cursor to top-left corner
    cout << "\x1B[2J\x1B[H";
}

//** main function in LiDAR IMU calibration **//
void Gril_Calib::LI_Calibration(int &orig_odom_freq, int &cut_frame_num, double &timediff_imu_wrt_lidar,
                                const double &move_start_time) {

    TimeConsuming time("Batch optimization");

    downsample_interpolate_IMU(move_start_time);    
    fout_before_filter();                           
    IMU_time_compensate(0.0, true); 

    deque<CalibState> IMU_after_zero_phase;
    deque<CalibState> Lidar_after_zero_phase;
    zero_phase_filt(get_IMU_state(), IMU_after_zero_phase); // zero phase low-pass filter
    normalize_acc(IMU_after_zero_phase);    
    zero_phase_filt(get_Lidar_state(), Lidar_after_zero_phase);
    set_IMU_state(IMU_after_zero_phase);
    set_Lidar_state(Lidar_after_zero_phase);
    cut_sequence_tail(); 

    xcorr_temporal_init(orig_odom_freq * cut_frame_num);
    IMU_time_compensate(get_lag_time_1(), false);


    central_diff(); 

    deque<CalibState> IMU_after_2nd_zero_phase;
    deque<CalibState> Lidar_after_2nd_zero_phase;
    zero_phase_filt(get_IMU_state(), IMU_after_2nd_zero_phase);
    zero_phase_filt(get_Lidar_state(), Lidar_after_2nd_zero_phase);

    
    set_states_2nd_filter(IMU_after_2nd_zero_phase, Lidar_after_2nd_zero_phase);    
    fout_check_lidar(); // file output for visualizing lidar low pass filter

   
    solve_Rotation_only();
    acc_interpolate();
    align_Group(IMU_state_group, Lidar_wrt_ground_group, IMU_wrt_ground_group,
                normal_vector_wrt_lidar_group, distance_Lidar_wrt_ground_group);

    // Calibration at once
    solve_Rot_Trans_calib(timediff_imu_wrt_lidar, imu_sensor_height);

    printf(BOLDBLUE"============================================================ \n\n" RESET);
    double time_L_I = timediff_imu_wrt_lidar + time_delay_IMU_wtr_Lidar;
    print_calibration_result(time_L_I, Rot_Lidar_wrt_IMU, Trans_Lidar_wrt_IMU, gyro_bias, acc_bias, Grav_L0);

    printf(BOLDBLUE"============================================================ \n\n" RESET);
    printf(BOLDCYAN "Gril-Calib : Ground Robot IMU-LiDAR calibration done.\n");
    printf("" RESET);
    
    // For debug
    // plot_result();
}

void Gril_Calib::print_calibration_result(double &time_L_I, M3D &R_L_I, V3D &p_L_I, V3D &bias_g, V3D &bias_a, V3D gravity) {
    cout.setf(ios::fixed);
    printf(BOLDCYAN "[Calibration Result] " RESET);
    cout << setprecision(6)
         << "Rotation matrix from LiDAR frame to IMU frame    = " << RotMtoEuler(R_L_I).transpose() * 57.3 << " deg" << endl;
    printf(BOLDCYAN "[Calibration Result] " RESET);
    cout << "Translation vector from LiDAR frame to IMU frame = " << p_L_I.transpose() << " m" << endl;
    printf(BOLDCYAN "[Calibration Result] " RESET);
    printf("Time Lag IMU to LiDAR    = %.8lf s \n", time_L_I);
    printf(BOLDCYAN "[Calibration Result] " RESET);
    cout << "Bias of Gyroscope        = " << bias_g.transpose() << " rad/s" << endl;
    printf(BOLDCYAN "[Calibration Result] " RESET);
    cout << "Bias of Accelerometer    = " << bias_a.transpose() << " m/s^2" << endl;
}

void Gril_Calib::plot_result() {
    vector<vector<double>> IMU_omg(3), IMU_acc(3), IMU_ang_acc(3), Lidar_omg(3), Lidar_acc(3), Lidar_ang_acc(3), Lidar_vel(3), Lidar_pos(3);
    for (auto it_IMU_state = IMU_state_group.begin(); it_IMU_state != IMU_state_group.end() - 1; it_IMU_state++) {
        for (int i = 0; i < 3; i++) {
            IMU_omg[i].push_back(it_IMU_state->ang_vel[i]);
            IMU_ang_acc[i].push_back(it_IMU_state->ang_acc[i]);
            IMU_acc[i].push_back(it_IMU_state->linear_acc[i]);
        }
    }
    for (auto it_Lidar_state = Lidar_state_group.begin();
         it_Lidar_state != Lidar_state_group.end() - 1; it_Lidar_state++) {
        for (int i = 0; i < 3; i++) {
            Lidar_pos[i].push_back(it_Lidar_state->pos_end[i]);
            Lidar_omg[i].push_back(it_Lidar_state->ang_vel[i]);
            Lidar_acc[i].push_back(it_Lidar_state->linear_acc[i]);
            Lidar_ang_acc[i].push_back(it_Lidar_state->ang_acc[i]);
            Lidar_vel[i].push_back(it_Lidar_state->linear_vel[i]);
        }
    }

    plt::figure(1);
    plt::subplot(2, 3, 1);
    plt::title("IMU Angular Velocity");
    plt::named_plot("IMU omg x", IMU_omg[0]);
    plt::named_plot("IMU omg y", IMU_omg[1]);
    plt::named_plot("IMU omg z", IMU_omg[2]);
    plt::legend();
    plt::grid(true);

    plt::subplot(2, 3, 2);
    plt::title("IMU Linear Acceleration");
    plt::named_plot("IMU acc x", IMU_acc[0]);
    plt::named_plot("IMU acc y", IMU_acc[1]);
    plt::named_plot("IMU acc z", IMU_acc[2]);
    plt::legend();
    plt::grid(true);

    plt::subplot(2, 3, 3);
    plt::title("IMU Angular Acceleration");
    plt::named_plot("IMU ang acc x", IMU_ang_acc[0]);
    plt::named_plot("IMU ang acc y", IMU_ang_acc[1]);
    plt::named_plot("IMU ang acc z", IMU_ang_acc[2]);
    plt::legend();
    plt::grid(true);

    plt::subplot(2, 3, 4);
    plt::title("LiDAR Angular Velocity");
    plt::named_plot("Lidar omg x", Lidar_omg[0]);
    plt::named_plot("Lidar omg y", Lidar_omg[1]);
    plt::named_plot("Lidar omg z", Lidar_omg[2]);
    plt::legend();
    plt::grid(true);

    plt::subplot(2, 3, 5);
    plt::title("LiDAR Linear Acceleration");
    plt::named_plot("Lidar acc x", Lidar_acc[0]);
    plt::named_plot("Lidar acc y", Lidar_acc[1]);
    plt::named_plot("Lidar acc z", Lidar_acc[2]);
    plt::legend();
    plt::grid(true);

    plt::subplot(2, 3, 6);
    plt::title("LiDAR Angular Acceleration");
    plt::named_plot("Lidar ang acc x", Lidar_ang_acc[0]);
    plt::named_plot("Lidar ang acc y", Lidar_ang_acc[1]);
    plt::named_plot("Lidar ang acc z", Lidar_ang_acc[2]);
    plt::legend();
    plt::grid(true);

    plt::show();
    plt::pause(0);
    plt::close();
}
