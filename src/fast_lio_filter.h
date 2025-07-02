#pragma once
#ifndef FAST_LIO_FILTER_H
#define FAST_LIO_FILTER_H

// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <memory>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <livox_ros_driver2/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <std_srvs/SetBool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <boost/make_shared.hpp>

constexpr double INIT_TIME = 0.1;
constexpr double LASER_POINT_COV = 0.001;
constexpr int MAXN = 720000;
constexpr int PUBFRAME_PERIOD = 20;

using namespace std;

class FastLioFilter
{
    public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FastLioFilter(const geometry_msgs::Pose& initial_pose = getZeroPose());
    ~FastLioFilter();

    // void SigHandle(int sig);
    static void SigHandle(int sig); // static version
    void handle_signal(int sig);    // instance method

    inline void dump_lio_state_to_log(FILE *fp);
    void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s);
    void pointBodyToWorld(PointType const * const pi, PointType * const po);
    template<typename T> void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po);
    void RGBpointBodyToWorld(PointType const * const pi, PointType * const po);
    void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po);
    void points_cache_collect();
    void lasermap_fov_segment();
    void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg);
    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in); 
    bool sync_packages(MeasureGroup &meas);
    void map_incremental();
    void publish_frame_world(const ros::Publisher & pubLaserCloudFull);
    void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body);
    void publish_effect_world(const ros::Publisher & pubLaserCloudEffect);
    void publish_map(const ros::Publisher & pubLaserCloudMap);
    template<typename T> void set_posestamp(T & out);
    void publish_odometry(const ros::Publisher & pubOdomAftMapped, const ros::Publisher & pubLioState);
    void publish_path(const ros::Publisher pubPath);

    static FastLioFilter* instance;
    void h_share_model_nonstatic(state_ikfom&, esekfom::dyn_share_datastruct<double>&);
    static void h_share_model(state_ikfom&, esekfom::dyn_share_datastruct<double>&);

    // void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
    int run_lio(ros::NodeHandle nh);
    void set_halt(bool halt);
    bool has_jumped(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2); 

    private:
    bool halt;
    double kdtree_incremental_time, kdtree_search_time, kdtree_delete_time;
    std::vector<double> T1, s_plot, s_plot2, s_plot3, s_plot4, s_plot5, s_plot6, s_plot7, s_plot8, s_plot9, s_plot10, s_plot11;    double match_time, solve_time, solve_const_H_time;
    int kdtree_size_st, kdtree_size_end, add_point_size, kdtree_delete_counter;
    bool runtime_pos_log, pcd_save_en, time_sync_en, extrinsic_est_en, path_en;
    
    //float res_last[100000];
    std::vector<float> res_last;
    float DET_RANGE;
    const float MOV_THRESHOLD;

    double time_diff_lidar_to_imu;

    std::mutex mtx_buffer;
    std::condition_variable sig_buffer;

    std::string root_dir;
    std::string map_file_path, lid_topic, imu_topic, init_frame, body_frame;

    double res_mean_last, total_residual;
    double last_timestamp_lidar, last_timestamp_imu;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_corner_min, filter_size_surf_min, filter_size_map_min, fov_deg;
    double cube_len, HALF_FOV_COS, FOV_DEG, total_distance, lidar_end_time, first_lidar_time;
    int effct_feat_num, time_log_counter, scan_count, publish_count;
    int iterCount, feats_down_size, NUM_MAX_ITERATIONS, laserCloudValidNum, pcd_save_interval, pcd_index;

    std::vector<bool> point_selected_surf;
    bool lidar_pushed, flg_first_scan, flg_EKF_inited;
    bool scan_pub_en, dense_pub_en, scan_body_pub_en, publish_tf;
    std::atomic<bool> flg_exit;

    int lidar_type;

    std::vector<std::vector<int>> pointSearchInd_surf;
    std::vector<BoxPointType> cub_needrm;
    std::vector<PointVector> Nearest_Points;
    std::vector<double> extrinT;
    std::vector<double> extrinR;
    std::deque<double> time_buffer;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

    PointCloudXYZI::Ptr featsFromMap;
    PointCloudXYZI::Ptr feats_undistort;
    PointCloudXYZI::Ptr feats_down_body;
    PointCloudXYZI::Ptr feats_down_world;
    PointCloudXYZI::Ptr normvec;
    PointCloudXYZI::Ptr laserCloudOri;
    PointCloudXYZI::Ptr corr_normvect;
    PointCloudXYZI::Ptr _featsArray;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    // KD_TREE<PointType> ikdtree;
    std::shared_ptr<KD_TREE<PointType>> ikdtree;

    V3F XAxisPoint_body;
    V3F XAxisPoint_world;
    V3D euler_cur;
    V3D position_last;
    V3D Lidar_T_wrt_IMU;
    M3D Lidar_R_wrt_IMU;

    MeasureGroup Measures;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state_point;
    vect3 pos_lid;

    nav_msgs::Path path;
    nav_msgs::Odometry odomAftMapped;
    nav_msgs::Odometry odomAftMappedPrv;
    geometry_msgs::Quaternion geoQuat;
    geometry_msgs::PoseStamped msg_body_pose;
    geometry_msgs::Pose initial_state;

    std::shared_ptr<Preprocess> p_pre;
    std::shared_ptr<ImuProcess> p_imu;

    BoxPointType LocalMap_Points;
    bool Localmap_Initialized; 

    double timediff_lidar_wrt_imu;
    bool   timediff_set_flg;

    double lidar_mean_scantime;
    int    scan_num;

    int process_increments;

    PointCloudXYZI::Ptr pcl_wait_pub;
    PointCloudXYZI::Ptr pcl_wait_save;

    bool check_for_jumps;
    double jump_position_threshold, jump_orientation_threshold;
    bool is_first_publish_odom;
    bool jump_detected;

    static geometry_msgs::Pose getZeroPose()
    {
        geometry_msgs::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        return pose;
    }
};
#endif