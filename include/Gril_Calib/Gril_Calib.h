#ifndef Gril_Calib_H
#define Gril_Calib_H

#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <condition_variable>
#include <sys/time.h>
#include <algorithm>
#include <csignal>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
// #include <eigen_conversions/eigen_msg.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/local_parameterization.h>
#include <ceres/covariance.h>

#include <so3_math.h>
#include "matplotlibcpp.h"
#include <common_lib.h>

#define FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))

namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;

typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Eigen::Quaterniond QD;
const V3D STD_GRAV = V3D(0, 0, -G_m_s2);

extern double GYRO_FACTOR_;
extern double ACC_FACTOR_;
extern double GROUND_FACTOR_;


// Lidar IMU calibration
// States needed by calibration
struct CalibState {
    M3D rot_end;
    V3D pos_end;
    V3D ang_vel;
    V3D linear_vel;
    V3D ang_acc;
    V3D linear_acc;
    double timeStamp;

    CalibState() {
        rot_end = Eye3d;
        pos_end = Zero3d;
        ang_vel = Zero3d;
        linear_vel = Zero3d;
        ang_acc = Zero3d;
        linear_acc = Zero3d;
        timeStamp = 0.0;
    };

    CalibState(const CalibState &b) {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->ang_vel = b.ang_vel;
        this->ang_acc = b.ang_acc;
        this->linear_vel = b.linear_vel;
        this->linear_acc = b.linear_acc;
        this->timeStamp = b.timeStamp;
    };

    CalibState operator*(const double &coeff) {
        CalibState a;
        a.ang_vel = this->ang_vel * coeff;
        a.ang_acc = this->ang_acc * coeff;
        a.linear_vel = this->linear_vel * coeff;
        a.linear_acc = this->linear_acc * coeff;
        return a;
    };

    CalibState &operator+=(const CalibState &b) {
        this->ang_vel += b.ang_vel;
        this->ang_acc += b.ang_acc;
        this->linear_vel += b.linear_vel;
        this->linear_acc += b.linear_acc;
        return *this;
    };

    CalibState &operator-=(const CalibState &b) {
        this->ang_vel -= b.ang_vel;
        this->ang_acc -= b.ang_acc;
        this->linear_vel -= b.linear_vel;
        this->linear_acc -= b.linear_acc;
        return *this;
    };

    CalibState &operator=(const CalibState &b) {
        this->ang_vel = b.ang_vel;
        this->ang_acc = b.ang_acc;
        this->linear_vel = b.linear_vel;
        this->linear_acc = b.linear_acc;
        return *this;
    };
};


struct Angular_Vel_Cost_only_Rot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Angular_Vel_Cost_only_Rot(V3D IMU_ang_vel_, V3D Lidar_ang_vel_) :
            IMU_ang_vel(IMU_ang_vel_), Lidar_ang_vel(Lidar_ang_vel_) {}

    template<typename T>
    bool operator()(const T *q, T *residual) const {
        Eigen::Matrix<T, 3, 1> IMU_ang_vel_T = IMU_ang_vel.cast<T>();
        Eigen::Matrix<T, 3, 1> Lidar_ang_vel_T = Lidar_ang_vel.cast<T>();
        Eigen::Quaternion<T> q_LI{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 3> R_LI = q_LI.toRotationMatrix();  //Rotation (from LiDAR to IMU)
        Eigen::Matrix<T, 3, 1> resi = R_LI * Lidar_ang_vel_T - IMU_ang_vel_T;
        residual[0] = resi[0];
        residual[1] = resi[1];
        residual[2] = resi[2];
        return true;
    }

    static ceres::CostFunction *Create(const V3D IMU_ang_vel_, const V3D Lidar_ang_vel_) {
        return (new ceres::AutoDiffCostFunction<Angular_Vel_Cost_only_Rot, 3, 4>(
                new Angular_Vel_Cost_only_Rot(IMU_ang_vel_, Lidar_ang_vel_)));
    }

    V3D IMU_ang_vel;
    V3D Lidar_ang_vel;
};

struct Angular_Vel_IL_Cost {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Angular_Vel_IL_Cost(V3D IMU_ang_vel_, V3D IMU_ang_acc_, V3D Lidar_ang_vel_, double deltaT_LI_) :
            IMU_ang_vel(IMU_ang_vel_), IMU_ang_acc(IMU_ang_acc_), Lidar_ang_vel(Lidar_ang_vel_),
            deltaT_LI(deltaT_LI_) {}

    template<typename T>
    bool operator()(const T *q, const T *b_g, const T *t, T *residual) const {
        //Known parameters used for Residual Construction
        Eigen::Matrix<T, 3, 1> IMU_ang_vel_T = IMU_ang_vel.cast<T>();
        Eigen::Matrix<T, 3, 1> IMU_ang_acc_T = IMU_ang_acc.cast<T>();
        Eigen::Matrix<T, 3, 1> Lidar_ang_vel_T = Lidar_ang_vel.cast<T>();
        T deltaT_LI_T{deltaT_LI};

        //Unknown Parameters, needed to be estimated
        Eigen::Quaternion<T> q_IL{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 3> R_IL = q_IL.toRotationMatrix();  //Rotation
        Eigen::Matrix<T, 3, 1> bias_g{b_g[0], b_g[1], b_g[2]};  //Bias of gyroscope
        T td{t[0]};                                             //Time lag (IMU wtr Lidar)

        // original Residual
        Eigen::Matrix<T, 3, 1> resi = R_IL.transpose() * Lidar_ang_vel_T - IMU_ang_vel_T - (deltaT_LI_T + td) * IMU_ang_acc_T + bias_g;
        residual[0] = T(GYRO_FACTOR_) * resi[0];
        residual[1] = T(GYRO_FACTOR_) * resi[1];
        residual[2] = T(GYRO_FACTOR_) * resi[2];
        return true;
    }

    static ceres::CostFunction *
    Create(const V3D IMU_ang_vel_, const V3D IMU_ang_acc_, const V3D Lidar_ang_vel_, const double deltaT_LI_) {
        // 3 residual, 4 parameter (q), 3 parameter (bias), 1 parameter (time lag)
        return (new ceres::AutoDiffCostFunction<Angular_Vel_IL_Cost, 3, 4, 3, 1>(
                new Angular_Vel_IL_Cost(IMU_ang_vel_, IMU_ang_acc_, Lidar_ang_vel_, deltaT_LI_)));
    }

    V3D IMU_ang_vel;
    V3D IMU_ang_acc;
    V3D Lidar_ang_vel;
    double deltaT_LI;
};

struct Angular_Vel_Cost {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Angular_Vel_Cost(V3D IMU_ang_vel_, V3D IMU_ang_acc_, V3D Lidar_ang_vel_, double deltaT_LI_) :
            IMU_ang_vel(IMU_ang_vel_), IMU_ang_acc(IMU_ang_acc_), Lidar_ang_vel(Lidar_ang_vel_),
            deltaT_LI(deltaT_LI_) {}

    template<typename T>
    bool operator()(const T *q, const T *b_g, const T *t, T *residual) const {
        //Known parameters used for Residual Construction
        Eigen::Matrix<T, 3, 1> IMU_ang_vel_T = IMU_ang_vel.cast<T>();
        Eigen::Matrix<T, 3, 1> IMU_ang_acc_T = IMU_ang_acc.cast<T>();
        Eigen::Matrix<T, 3, 1> Lidar_ang_vel_T = Lidar_ang_vel.cast<T>();
        T deltaT_LI_T{deltaT_LI};

        //Unknown Parameters, needed to be estimated
        Eigen::Quaternion<T> q_LI{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 3> R_LI = q_LI.toRotationMatrix();  //Rotation
        Eigen::Matrix<T, 3, 1> bias_g{b_g[0], b_g[1], b_g[2]};  //Bias of gyroscope
        T td{t[0]};                                             //Time lag (IMU wtr Lidar)

        // Residual
        Eigen::Matrix<T, 3, 1> resi =
                R_LI * Lidar_ang_vel_T - IMU_ang_vel_T - (deltaT_LI_T + td) * IMU_ang_acc_T + bias_g;
        
        residual[0] = resi[0];
        residual[1] = resi[1];
        residual[2] = resi[2];
        return true;
    }

    static ceres::CostFunction *
    Create(const V3D IMU_ang_vel_, const V3D IMU_ang_acc_, const V3D Lidar_ang_vel_, const double deltaT_LI_) {
        // 3 residual, 4 parameter (q), 3 parameter (bias), 1 parameter (time lag)
        return (new ceres::AutoDiffCostFunction<Angular_Vel_Cost, 3, 4, 3, 1>(
                new Angular_Vel_Cost(IMU_ang_vel_, IMU_ang_acc_, Lidar_ang_vel_, deltaT_LI_)));
    }

    V3D IMU_ang_vel;
    V3D IMU_ang_acc;
    V3D Lidar_ang_vel;
    double deltaT_LI;
};



struct Ground_Plane_Cost_IL {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Ground_Plane_Cost_IL(QD Lidar_wrt_ground_, QD IMU_wrt_ground_,  
                         const double distance_Lidar_wrt_ground_, const double imu_sensor_height_) :
            Lidar_wrt_ground(Lidar_wrt_ground_), IMU_wrt_ground(IMU_wrt_ground_),  
                        distance_Lidar_wrt_ground(distance_Lidar_wrt_ground_), imu_sensor_height(imu_sensor_height_) {}

    template<typename T>
    bool operator()(const T *q, const T *trans, T *residual) const {
        // Known parameters used for Residual Construction
        Eigen::Quaternion<T> Lidar_wrt_ground_T = Lidar_wrt_ground.cast<T>();           // from ground frame to LiDAR plane frame
        Eigen::Matrix<T, 3, 3> R_GL = Lidar_wrt_ground_T.toRotationMatrix();            // Rotation matrix (from Ground to LiDAR)
        Eigen::Quaternion<T> IMU_wrt_ground_T = IMU_wrt_ground.cast<T>();               // from ground frame (earth frame) to IMU frame
        Eigen::Matrix<T, 3, 3> R_GI = IMU_wrt_ground_T.toRotationMatrix();              // Rotation matrix (from Ground to IMU)
       
        T distance_Lidar_wrt_ground_T = T(distance_Lidar_wrt_ground);                   // Distance of LiDAR plane from ground
        T imu_sensor_height_T = T(imu_sensor_height);                                   // Height of IMU sensor from ground

        // Unknown Parameters, needed to be estimated
        Eigen::Quaternion<T> q_IL{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 3> R_IL = q_IL.toRotationMatrix();  // Rotation IMU frame to LiDAR frame
        Eigen::Matrix<T, 3, 1> T_IL{trans[0], trans[1], trans[2]}; // Translation of I-L (IMU wtr Lidar)
        
        // Plane motion constraint Residual
        Eigen::Matrix<T, 3, 3> R_plane = R_IL.transpose() * R_GI.transpose() * R_GL;
        Eigen::Matrix<T, 3, 1> e3 = Eigen::Matrix<T, 3, 1>::UnitZ();    // (0,0,1)
        Eigen::Matrix<T, 3, 1> resi_plane = R_plane * e3;

        // Distance constraint Residual - TODO : Generalized
        Eigen::Matrix<T, 3, 1> imu_height_vec = imu_sensor_height_T * e3;
        Eigen::Matrix<T, 3, 1> lidar_height_vec = distance_Lidar_wrt_ground_T * e3;
        Eigen::Matrix<T, 3, 1> tmp_distance = R_IL * R_GI * imu_height_vec - R_GL * lidar_height_vec;

        T resi_distance = T_IL[2] - tmp_distance[2];

        // Residual
        residual[0] = T(GROUND_FACTOR_) * resi_plane[0]; 
        residual[1] = T(GROUND_FACTOR_) * resi_plane[1]; 
        residual[2] = T(GROUND_FACTOR_) * resi_distance;

        return true;
    }

    static ceres::CostFunction *
    Create(const QD Lidar_wrt_ground_, const QD IMU_wrt_ground_, 
           const double distance_Lidar_wrt_ground, const double imu_sensor_height) {
        // 3 residual, 4 parameter (q), 3 paramter (translation)
        return (new ceres::AutoDiffCostFunction<Ground_Plane_Cost_IL, 3, 4, 3>(
                new Ground_Plane_Cost_IL(Lidar_wrt_ground_, IMU_wrt_ground_, 
                                         distance_Lidar_wrt_ground, imu_sensor_height)));
    }

    QD Lidar_wrt_ground;
    QD IMU_wrt_ground;
    double distance_Lidar_wrt_ground;
    double imu_sensor_height;
};

struct Linear_acc_Rot_Cost_without_Gravity {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Linear_acc_Rot_Cost_without_Gravity(CalibState LidarState_, V3D IMU_linear_acc_, QD Lidar_wrt_ground_) :
            LidarState(LidarState_), IMU_linear_acc(IMU_linear_acc_), Lidar_wrt_ground(Lidar_wrt_ground_) {}

    template<typename T>
    bool operator()(const T *q,const T *b_a, const T *trans, T *residual) const {
        // Known parameters used for Residual Construction
        Eigen::Matrix<T, 3, 3> R_LL0_T = LidarState.rot_end.cast<T>();  
        Eigen::Matrix<T, 3, 1> IMU_linear_acc_T = IMU_linear_acc.cast<T>();             // Linear acceleration of IMU
        Eigen::Matrix<T, 3, 1> Lidar_linear_acc_T = LidarState.linear_acc.cast<T>();    // lidar linear acceleration
        Eigen::Quaternion<T> Lidar_wrt_ground_T = Lidar_wrt_ground.cast<T>();           // from ground frame to LiDAR plane frame
        Eigen::Matrix<T, 3, 3> R_GL = Lidar_wrt_ground_T.toRotationMatrix();            // Rotation matrix from Ground frame to LiDAR frame

        // Unknown Parameters, needed to be estimated
        Eigen::Matrix<T, 3, 1> bias_aL{b_a[0], b_a[1], b_a[2]};    // Bias of Linear acceleration
        Eigen::Matrix<T, 3, 1> T_IL{trans[0], trans[1], trans[2]}; // Translation of I-L (IMU wtr Lidar)
        Eigen::Quaternion<T> q_IL{q[0], q[1], q[2], q[3]};         // Rotation from IMU frame to LiDAR frame
        Eigen::Matrix<T, 3, 3> R_IL = q_IL.toRotationMatrix();

        // Residual Construction
        M3D Lidar_omg_SKEW, Lidar_angacc_SKEW;
        Lidar_omg_SKEW << SKEW_SYM_MATRX(LidarState.ang_vel);
        Lidar_angacc_SKEW << SKEW_SYM_MATRX(LidarState.ang_acc);

        M3D Jacob_trans = Lidar_omg_SKEW * Lidar_omg_SKEW + Lidar_angacc_SKEW;
        Eigen::Matrix<T, 3, 3> Jacob_trans_T = Jacob_trans.cast<T>();
        
        // 이 부분에서 R_LL0_T * Jacob_trans_T * T_IL;의 부호가 반대로 되야 값이 잘 나온다. 이유 분석을 해야함
        Eigen::Matrix<T, 3, 1> resi = R_LL0_T * R_IL * IMU_linear_acc_T - R_LL0_T * bias_aL
                                      + R_GL * STD_GRAV - Lidar_linear_acc_T - R_LL0_T * Jacob_trans_T * T_IL;

        
        residual[0] = T(ACC_FACTOR_) * resi[0];
        residual[1] = T(ACC_FACTOR_) * resi[1];
        residual[2] = T(ACC_FACTOR_) * resi[2];
        
        return true;
    }

    static ceres::CostFunction *Create(const CalibState LidarState_, const V3D IMU_linear_acc_, const QD Lidar_wrt_ground_) {
        // residual 3, quaternion 4, bias 3, translation 3
        return (new ceres::AutoDiffCostFunction<Linear_acc_Rot_Cost_without_Gravity, 3, 4, 3, 3>(
                new Linear_acc_Rot_Cost_without_Gravity(LidarState_, IMU_linear_acc_, Lidar_wrt_ground_)));
    }

    CalibState LidarState;
    V3D IMU_linear_acc;
    QD Lidar_wrt_ground;
};

class Gril_Calib {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ofstream fout_LiDAR_meas, fout_IMU_meas, fout_before_filt_IMU, fout_before_filt_Lidar, fout_acc_cost, fout_after_rot;
    ofstream fout_LiDAR_ang_vel, fout_IMU_ang_vel, fout_Jacob_trans;
    ofstream fout_LiDAR_meas_after;

    double data_accum_length = 0.0;
    double x_accumulate = 0.0;
    double y_accumulate = 0.0;
    double z_accumulate = 0.0;
    double svd_threshold = 0.01;
    double imu_sensor_height = 0.0;
    double trans_IL_x, trans_IL_y, trans_IL_z; 
    
    double bound_th;
    bool set_boundary = false;
    bool verbose = false;


    Gril_Calib();

    ~Gril_Calib();

    // original
    struct Butterworth {
        // Coefficients of 6 order butterworth low pass filter, omega = 0.15 -성공
        double Coeff_b[7] = {0.0001,0.0005,0.0011,0.0015,0.0011,0.0005,0.0001};
        double Coeff_a[7] = {1,-4.1824,7.4916,-7.3136,4.0893,-1.2385,0.1584};

        int Coeff_size = 7;
        int extend_num = 0;
    };

    void plot_result();

    void push_ALL_IMU_CalibState(const sensor_msgs::msg::Imu::SharedPtr &msg, const double &mean_acc_norm);

    void push_IMU_CalibState(const V3D &omg, const V3D &acc, const double &timestamp);

    void push_Lidar_CalibState(const M3D &rot, const V3D &pos, const V3D &omg, const V3D &linear_vel, const double &timestamp);

    void push_Plane_Constraint(const Eigen::Quaterniond &q_lidar, const Eigen::Quaterniond &q_imu, const V3D &normal_lidar, const double &distance_lidar);

    void downsample_interpolate_IMU(const double &move_start_time);

    void central_diff();

    void xcorr_temporal_init(const double &odom_freq);

    void IMU_time_compensate(const double &lag_time, const bool &is_discard);

    void acc_interpolate();

    void Butter_filt(const deque<CalibState> &signal_in, deque<CalibState> &signal_out);

    void zero_phase_filt(const deque<CalibState> &signal_in, deque<CalibState> &signal_out);

    void cut_sequence_tail();

    void set_IMU_state(const deque<CalibState> &IMU_states);

    void set_Lidar_state(const deque<CalibState> &Lidar_states);

    void set_states_2nd_filter(const deque<CalibState> &IMU_states, const deque<CalibState> &Lidar_states);

    void solve_Rot_Trans_calib(double &timediff_imu_wrt_lidar, const double &imu_sensor_height);

    void normalize_acc(deque<CalibState> &signal_in);

    void align_Group(const deque<CalibState> &IMU_states, deque<Eigen::Quaterniond> &Lidar_wrt_ground_states, 
                           deque<Eigen::Quaterniond> &IMU_wrt_ground_states, deque<V3D> &normal_vector_wrt_lidar_group, 
                           deque<double> &distance_Lidar_ground_states);

    bool data_sufficiency_assess(MatrixXd &Jacobian_rot, int &frame_num, V3D &lidar_omg, int &orig_odom_freq, int &cut_frame_num, QD &lidar_q, QD &imu_q, double &lidar_estimate_height);

    void solve_Rotation_only();

    void printProgress(double percentage, int axis);

    void clear();

    void fout_before_filter();

    void fout_check_lidar();

    void LI_Calibration(int &orig_odom_freq, int &cut_frame_num, double &timediff_imu_wrt_lidar, const double &move_start_time);

    void print_calibration_result(double &time_L_I, M3D &R_L_I, V3D &p_L_I, V3D &bias_g, V3D &bias_a, V3D gravity);

    inline double get_lag_time_1() {
        return time_lag_1;
    }

    inline double get_lag_time_2() {
        return time_lag_2;
    }

    inline double get_total_time_lag() {
        return time_delay_IMU_wtr_Lidar;
    }

    inline double get_time_result() {
        return time_offset_result;
    }

    inline V3D get_Grav_L0() {
        return Grav_L0;
    }

    inline M3D get_R_LI() {
        return Rot_Lidar_wrt_IMU;
    }

    inline V3D get_T_LI() {
        return Trans_Lidar_wrt_IMU;
    }

    inline V3D get_gyro_bias() {
        return gyro_bias;
    }

    inline V3D get_acc_bias() {
        return acc_bias;
    }

    inline void IMU_buffer_clear() {
        IMU_state_group_ALL.clear();
    }

    deque<CalibState> get_IMU_state() {
        return IMU_state_group;
    }

    deque<CalibState> get_Lidar_state() {
        return Lidar_state_group;
    }

private:
    deque<CalibState> IMU_state_group;      // LiDAR와 interpolation을 진행한 IMU data 결과를 가지고 있는 groups
    deque<CalibState> Lidar_state_group;    // LiDAR state 
    deque<CalibState> IMU_state_group_ALL;  // 모든 IMU data (ROS topic)를 가지고 있는 groups
    deque<Eigen::Quaterniond> Lidar_wrt_ground_group; // from ground frame (earth frame) to LiDAR plane frame
    deque<Eigen::Quaterniond> IMU_wrt_ground_group; // from ground frame (earth frame) to IMU frame
    deque<V3D> normal_vector_wrt_lidar_group;         // normal vector of LiDAR ground segmentation
    deque<double> distance_Lidar_wrt_ground_group;  // distance from ground frame (earth frame) to LiDAR frame
    V3D Grav_L0;                  // Gravity vector in the initial Lidar frame L_0
  
    /// Parameters needed to be calibrated
    M3D Rot_Grav_wrt_Init_Lidar;  // Rotation from inertial frame G to initial Lidar frame L_0
    M3D Rot_Lidar_wrt_IMU;        // Rotation from Lidar frame L to IMU frame I
    V3D Trans_Lidar_wrt_IMU;      // Translation from Lidar frame L to IMU frame I
    V3D gyro_bias;                // gyro bias
    V3D acc_bias;                 // acc bias

    double time_delay_IMU_wtr_Lidar; //(Soft) time delay between IMU and Lidar = time_lag_1 + time_lag_2
    double time_offset_result;       // Time offset between IMU and Lidar (final result)
    double time_lag_1;               // Time offset estimated by cross-correlation
    double time_lag_2;               // Time offset estimated by unified optimization
    int lag_IMU_wtr_Lidar;           // positive: timestamp of IMU is larger than that of LiDAR

    // Previous calibration results
    M3D R_IL_prev;                      // Rotation from Lidar frame L to IMU frame I
    V3D T_IL_prev;                      // Translation from Lidar frame L to IMU frame I
    V3D gyro_bias_prev;                 // gyro bias
    V3D acc_bias_prev;                  // acc bias
    double time_lag_2_prev;             // Time offset estimated by unified optimization

};

#endif