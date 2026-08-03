#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <vector>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "use-ikfom.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

/* === BEGIN FAST_LIO_INIT_DIAG (instrumentation only; no estimator math changes) === */
struct InitDiagSample {
  double stamp_s = 0.0;
  int sample_count = 0;
  double mean_acc_x = 0.0, mean_acc_y = 0.0, mean_acc_z = 0.0;
  double mean_gyr_x = 0.0, mean_gyr_y = 0.0, mean_gyr_z = 0.0;
  double mean_acc_norm = 0.0;
  double grav_x = 0.0, grav_y = 0.0, grav_z = 0.0;
  double bg_x = 0.0, bg_y = 0.0, bg_z = 0.0;
};

struct InitDiagState {
  bool enabled = false;
  bool completed = false;
  bool first_lidar_logged = false;
  double first_imu_stamp_s = -1.0;
  double init_start_stamp_s = -1.0;
  double init_complete_stamp_s = -1.0;
  double first_lidar_after_init_s = -1.0;
  double bag_relative_init_complete_s = -1.0;
  int imu_samples_used = 0;
  V3D final_grav = Zero3d;
  V3D final_bg = Zero3d;
  V3D final_ba = Zero3d;
  V3D final_mean_acc = Zero3d;
  V3D final_mean_gyr = Zero3d;
  double final_mean_acc_norm = 0.0;
  std::vector<InitDiagSample> convergence;
};

static InitDiagState g_init_diag;

static void init_diag_refresh_enabled()
{
  const char *path = std::getenv("FAST_LIO_INIT_DIAG_PATH");
  g_init_diag.enabled = (path != nullptr && path[0] != '\0');
}

static void init_diag_note_imu_stamp(double stamp_s)
{
  if (!g_init_diag.enabled) return;
  if (g_init_diag.first_imu_stamp_s < 0.0) {
    g_init_diag.first_imu_stamp_s = stamp_s;
  }
}

static void init_diag_record_step(
    double stamp_s, int sample_count, const V3D &mean_acc, const V3D &mean_gyr,
    const state_ikfom &imu_state)
{
  if (!g_init_diag.enabled) return;
  InitDiagSample s;
  s.stamp_s = stamp_s;
  s.sample_count = sample_count;
  s.mean_acc_x = mean_acc(0); s.mean_acc_y = mean_acc(1); s.mean_acc_z = mean_acc(2);
  s.mean_gyr_x = mean_gyr(0); s.mean_gyr_y = mean_gyr(1); s.mean_gyr_z = mean_gyr(2);
  s.mean_acc_norm = mean_acc.norm();
  s.grav_x = imu_state.grav[0]; s.grav_y = imu_state.grav[1]; s.grav_z = imu_state.grav[2];
  s.bg_x = imu_state.bg[0]; s.bg_y = imu_state.bg[1]; s.bg_z = imu_state.bg[2];
  g_init_diag.convergence.push_back(s);
}

static void init_diag_write_json()
{
  const char *path = std::getenv("FAST_LIO_INIT_DIAG_PATH");
  if (!path || path[0] == '\0') return;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9);
  oss << "{\n"
      << "  \"first_imu_stamp_s\": " << g_init_diag.first_imu_stamp_s << ",\n"
      << "  \"init_start_stamp_s\": " << g_init_diag.init_start_stamp_s << ",\n"
      << "  \"init_complete_stamp_s\": " << g_init_diag.init_complete_stamp_s << ",\n"
      << "  \"first_lidar_after_init_s\": " << g_init_diag.first_lidar_after_init_s << ",\n"
      << "  \"imu_samples_used\": " << g_init_diag.imu_samples_used << ",\n"
      << "  \"max_ini_count\": " << MAX_INI_COUNT << ",\n"
      << "  \"final_mean_acc\": [" << g_init_diag.final_mean_acc(0) << ", "
      << g_init_diag.final_mean_acc(1) << ", " << g_init_diag.final_mean_acc(2) << "],\n"
      << "  \"final_mean_gyr\": [" << g_init_diag.final_mean_gyr(0) << ", "
      << g_init_diag.final_mean_gyr(1) << ", " << g_init_diag.final_mean_gyr(2) << "],\n"
      << "  \"final_mean_acc_norm\": " << g_init_diag.final_mean_acc_norm << ",\n"
      << "  \"final_gravity\": [" << g_init_diag.final_grav(0) << ", "
      << g_init_diag.final_grav(1) << ", " << g_init_diag.final_grav(2) << "],\n"
      << "  \"final_gyro_bias\": [" << g_init_diag.final_bg(0) << ", "
      << g_init_diag.final_bg(1) << ", " << g_init_diag.final_bg(2) << "],\n"
      << "  \"final_accel_bias\": [" << g_init_diag.final_ba(0) << ", "
      << g_init_diag.final_ba(1) << ", " << g_init_diag.final_ba(2) << "],\n"
      << "  \"accel_bias_initialized\": false,\n"
      << "  \"convergence\": [\n";
  for (size_t i = 0; i < g_init_diag.convergence.size(); ++i) {
    const auto &c = g_init_diag.convergence[i];
    oss << "    {\"stamp_s\": " << c.stamp_s
        << ", \"sample_count\": " << c.sample_count
        << ", \"mean_acc\": [" << c.mean_acc_x << ", " << c.mean_acc_y << ", " << c.mean_acc_z << "]"
        << ", \"mean_gyr\": [" << c.mean_gyr_x << ", " << c.mean_gyr_y << ", " << c.mean_gyr_z << "]"
        << ", \"mean_acc_norm\": " << c.mean_acc_norm
        << ", \"gravity\": [" << c.grav_x << ", " << c.grav_y << ", " << c.grav_z << "]"
        << ", \"gyro_bias\": [" << c.bg_x << ", " << c.bg_y << ", " << c.bg_z << "]}";
    if (i + 1 < g_init_diag.convergence.size()) oss << ",";
    oss << "\n";
  }
  oss << "  ]\n}\n";
  std::string tmp = std::string(path) + ".tmp";
  {
    std::ofstream ofs(tmp.c_str(), std::ios::out | std::ios::trunc);
    if (ofs) ofs << oss.str();
  }
  std::rename(tmp.c_str(), path);
}

static void init_diag_print_summary()
{
  if (!g_init_diag.enabled || !g_init_diag.completed) return;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9);
  oss << "\n========== FAST_LIO_INIT_DIAG_SUMMARY ==========\n"
      << "first_imu_stamp_s: " << g_init_diag.first_imu_stamp_s << "\n"
      << "init_start_stamp_s: " << g_init_diag.init_start_stamp_s << "\n"
      << "init_complete_stamp_s: " << g_init_diag.init_complete_stamp_s << "\n"
      << "first_lidar_after_init_s: " << g_init_diag.first_lidar_after_init_s << "\n"
      << "imu_samples_used: " << g_init_diag.imu_samples_used << "\n"
      << "final_gravity: " << g_init_diag.final_grav.transpose() << "\n"
      << "final_gyro_bias: " << g_init_diag.final_bg.transpose() << "\n"
      << "final_accel_bias: " << g_init_diag.final_ba.transpose() << "\n"
      << "final_mean_acc_norm: " << g_init_diag.final_mean_acc_norm << "\n"
      << "================================================\n";
  std::cerr << oss.str() << std::flush;
}
/* === END FAST_LIO_INIT_DIAG === */

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  // sensor_msgs::ImuConstPtr last_imu_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  init_diag_refresh_enabled();
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess() {
  init_diag_write_json();
  init_diag_print_summary();
}

void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
    if (g_init_diag.enabled) {
      g_init_diag.init_start_stamp_s = rclcpp::Time(meas.imu.front()->header.stamp).seconds();
    }
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    init_diag_note_imu_stamp(rclcpp::Time(imu->header.stamp).seconds());

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);
  init_diag_record_step(
      !meas.imu.empty() ? rclcpp::Time(meas.imu.back()->header.stamp).seconds() : first_lidar_time,
      N - 1, mean_acc, mean_gyr, init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = rclcpp::Time(v_imu.front()->header.stamp).seconds();
  const double &imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;
  
  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    double tail_stamp = rclcpp::Time(tail->header.stamp).seconds();
    double head_stamp = rclcpp::Time(head->header.stamp).seconds();

    if (tail_stamp < last_lidar_end_time_)    continue;
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head_stamp < last_lidar_end_time_)
    {
      dt = tail_stamp - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail_stamp - head_stamp;
    }
    
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail_stamp - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);
    pos_imu<<VEC_FROM_ARRAY(head->pos);
    acc_imu<<VEC_FROM_ARRAY(tail->acc);
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(angvel_avr, dt));
      
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
      
      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  assert(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      std::cout << "IMU Initial Done" << std::endl;
      if (g_init_diag.enabled) {
        g_init_diag.completed = true;
        g_init_diag.imu_samples_used = init_iter_num;
        g_init_diag.init_complete_stamp_s = rclcpp::Time(last_imu_->header.stamp).seconds();
        g_init_diag.final_mean_acc = mean_acc;
        g_init_diag.final_mean_gyr = mean_gyr;
        g_init_diag.final_mean_acc_norm = mean_acc.norm();
        g_init_diag.final_grav = V3D(imu_state.grav[0], imu_state.grav[1], imu_state.grav[2]);
        g_init_diag.final_bg = imu_state.bg;
        g_init_diag.final_ba = imu_state.ba;
        init_diag_write_json();
      }
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  if (g_init_diag.enabled && g_init_diag.completed && !g_init_diag.first_lidar_logged) {
    g_init_diag.first_lidar_after_init_s = meas.lidar_beg_time;
    g_init_diag.first_lidar_logged = true;
    init_diag_write_json();
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
