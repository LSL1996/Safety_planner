#include "plan_env/sdf_map.h"

// #define current_img_ depth_image_[image_cnt_ & 1]
// #define last_img_ depth_image_[!(image_cnt_ & 1)]

void SDFMap::initMap(ros::NodeHandle& nh) { 
  /* get parameter */
  double x_size, y_size, z_size;
  nh.param("sdf_map/resolution", mp_.resolution_, -1.0);
  nh.param("sdf_map/map_size_x", x_size, -1.0);
  nh.param("sdf_map/map_size_y", y_size, -1.0);
  nh.param("sdf_map/map_size_z", z_size, -1.0);
  nh.param("sdf_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  nh.param("sdf_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  nh.param("sdf_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  nh.param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  nh.param("sdf_map/fx", mp_.fx_, -1.0);
  nh.param("sdf_map/fy", mp_.fy_, -1.0);
  nh.param("sdf_map/cx", mp_.cx_, -1.0);
  nh.param("sdf_map/cy", mp_.cy_, -1.0);

  nh.param("sdf_map/use_depth_filter", mp_.use_depth_filter_, true);
  nh.param("sdf_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  nh.param("sdf_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  nh.param("sdf_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  nh.param("sdf_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  nh.param("sdf_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  nh.param("sdf_map/skip_pixel", mp_.skip_pixel_, -1);

  nh.param("sdf_map/p_hit", mp_.p_hit_, 0.70);
  nh.param("sdf_map/p_miss", mp_.p_miss_, 0.35);
  nh.param("sdf_map/p_min", mp_.p_min_, 0.12);
  nh.param("sdf_map/p_max", mp_.p_max_, 0.97);
  nh.param("sdf_map/p_occ", mp_.p_occ_, 0.80);
  nh.param("sdf_map/min_ray_length", mp_.min_ray_length_, -0.1);
  nh.param("sdf_map/max_ray_length", mp_.max_ray_length_, -0.1);

  nh.param("sdf_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);
  nh.param("sdf_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  nh.param("sdf_map/ceil_height", mp_.ceil_height_, -0.1);

  nh.param("sdf_map/show_occ_time", mp_.show_occ_time_, false);
  nh.param("sdf_map/show_esdf_time", mp_.show_esdf_time_, false);
  nh.param("sdf_map/pose_type", mp_.pose_type_, 1);

  nh.param("sdf_map/frame_id", mp_.frame_id_, string("world"));
  nh.param("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);
  nh.param("sdf_map/local_map_margin", mp_.local_map_margin_, 1);
  nh.param("sdf_map/ground_height", mp_.ground_height_, 1.0);

  mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  occupancy_buffer_neg_ = vector<char>(buffer_size, 0);
  occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  distance_buffer_ = vector<double>(buffer_size, 10000);
  distance_buffer_neg_ = vector<double>(buffer_size, 10000);
  distance_buffer_all_ = vector<double>(buffer_size, 10000);

  count_hit_and_miss_ = vector<short>(buffer_size, 0);
  count_hit_ = vector<short>(buffer_size, 0);
  flag_rayend_ = vector<char>(buffer_size, -1);
  flag_traverse_ = vector<char>(buffer_size, -1);

  tmp_buffer1_ = vector<double>(buffer_size, 0);
  tmp_buffer2_ = vector<double>(buffer_size, 0);
  raycast_num_ = 0;

  proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  proj_points_cnt = 0;

  cam2body_ << 0.0, 0.0, 1.0, 0.0,
              -1.0, 0.0, 0.0, 0.0,
              0.0, -1.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 1.0;

  /* init callback */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/sdf_map/depth", 50));

  if (mp_.pose_type_ == POSE_STAMPED) {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "/sdf_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&SDFMap::depthPoseCallback, this, _1, _2));

  } else if (mp_.pose_type_ == ODOMETRY) {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/sdf_map/odom", 100));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&SDFMap::depthOdomCallback, this, _1, _2));
  }

  // use odometry and point cloud

  indep_cloud_sub_ =
      nh.subscribe<sensor_msgs::PointCloud2>("/sdf_map/cloud", 10, &SDFMap::cloudCallback, this);
  indep_odom_sub_ =
      nh.subscribe<nav_msgs::Odometry>("/sdf_map/odom", 10, &SDFMap::odomCallback, this);

  occ_timer_ = nh.createTimer(ros::Duration(0.05), &SDFMap::updateOccupancyCallback, this);
  esdf_timer_ = nh.createTimer(ros::Duration(0.05), &SDFMap::updateESDFCallback, this);
  vis_timer_ = nh.createTimer(ros::Duration(0.05), &SDFMap::visCallback, this);

  map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy", 10);
  map_inf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate", 10);
  esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = nh.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);

  unknown_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  depth_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);

  occ_need_update_ = false;
  need_clear_local_map_ = false;
  esdf_need_update_ = false;
  has_first_depth_ = false;
  has_odom_ = false;
  has_cloud_ = false;
  image_cnt_ = 0;

  esdf_time_ = 0.0;
  fuse_time_ = 0.0;
  update_num_ = 0;
  max_esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
}

void SDFMap::resetBuffer() {
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  local_bound_min_ = Eigen::Vector3i::Zero();
  local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        distance_buffer_[toAddress(x, y, z)] = 10000;
      }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_.map_voxel_num_(dim)];
  double z[mp_.map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = local_bound_min_;
  Eigen::Vector3i max_esdf = local_bound_max_;

  /* ========== compute positive DT ========== */

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 //  min(mp_.resolution_ * std::sqrt(val),
                 //      distance_buffer_[toAddress(x, y, z)]);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        if (occupancy_buffer_inflate_[idx] == 0) {
          occupancy_buffer_neg_[idx] = 1;

        } else if (occupancy_buffer_inflate_[idx] == 1) {
          occupancy_buffer_neg_[idx] = 0;
        } else {
          ROS_ERROR("what?");
        }
      }

  tmp_buffer1_.clear();
  tmp_buffer2_.clear();

  ros::Time t1, t2;

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return occupancy_buffer_neg_[x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                            y * mp_.map_voxel_num_(2) + z] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        distance_buffer_all_[idx] = distance_buffer_[idx];

        if (distance_buffer_neg_[idx] > 0.0)
          distance_buffer_all_[idx] += (-distance_buffer_neg_[idx] + mp_.resolution_);
      }
}

int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ) {
  if (occ != 1 && occ != 0) return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  count_hit_and_miss_[idx_ctns] += 1;

  if (count_hit_and_miss_[idx_ctns] == 1) {
    cache_voxel_.push(id);
  }

  if (occ == 1) count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void SDFMap::projectDepthImage() {
  // proj_points_.clear();
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  int cols = depth_image_.cols;
  int rows = depth_image_.rows;

  double depth;

  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();

  // cout << "rotate: " << camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << camera_pos_ << std::endl;

  if (!mp_.use_depth_filter_) {
    for (int v = 0; v < rows; v++) {
      row_ptr = depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u++) {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        
        // debug
        if (depth <= 0.1) continue; 

        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + camera_pos_;

        // if (u == 320 && v == 240) std::cout << "depth: " << depth << std::endl;
        proj_points_[proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else {

    if (!has_first_depth_)
      has_first_depth_ = true;
    else {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = last_camera_q_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_) {
        row_ptr = depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_) {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth with low certainty
          if (*row_ptr == 0) {
            depth = mp_.max_ray_length_ + 0.1;
          } else if (depth < mp_.depth_filter_mindist_) {
            continue;
          } else if (depth > mp_.depth_filter_maxdist_) {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, camera_pos_);
          // }

          proj_points_[proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false) {
            pt_reproj = last_camera_r_inv * (pt_world - last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows) {
              if (fabs(last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_) {
                proj_points_[proj_points_cnt++] = pt_world;
              }
            } else {
              proj_points_[proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  last_camera_pos_ = camera_pos_;
  last_camera_q_ = camera_q_;
  last_depth_image_ = depth_image_;
}

void SDFMap::raycastProcess() {
  // if (proj_points_.size() == 0)
  if (proj_points_cnt == 0) return;

  ros::Time t1, t2;

  raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < proj_points_cnt; ++i) {
    pt_w = proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w)) {
      pt_w = closetPointInMap(pt_w, camera_pos_);

      length = (pt_w - camera_pos_).norm();
      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - camera_pos_) / length * mp_.max_ray_length_ + camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);

    } else {
      length = (pt_w - camera_pos_).norm();

      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - camera_pos_) / length * mp_.max_ray_length_ + camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      } else {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX) {
      if (flag_rayend_[vox_idx] == raycast_num_) {
        continue;
      } else {
        flag_rayend_[vox_idx] = raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt)) {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX) {
        if (flag_traverse_[vox_idx] == raycast_num_) {
          break;
        } else {
          flag_traverse_[vox_idx] = raycast_num_;
        }
      }
    }
  }

  // determine the local bounding box for updating ESDF
  min_x = min(min_x, camera_pos_(0));
  min_y = min(min_y, camera_pos_(1));
  min_z = min(min_z, camera_pos_(2));

  max_x = max(max_x, camera_pos_(0));
  max_y = max(max_y, camera_pos_(1));
  max_z = max(max_z, camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), local_bound_min_);

  int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
  local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
  boundIndex(local_bound_min_);
  boundIndex(local_bound_max_);

  need_clear_local_map_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << cache_voxel_.size() << std::endl;

  while (!cache_voxel_.empty()) {

    Eigen::Vector3i idx = cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    cache_voxel_.pop();

    double log_odds_update =
        count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - count_hit_[idx_ctns] ?
        mp_.prob_hit_log_ :
        mp_.prob_miss_log_;

    count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
        idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local) {
      occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    occupancy_buffer_[idx_ctns] =
        std::min(std::max(occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap() {
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

      for (int z = min_cut_m(2); z < min_cut(2); ++z) {
        int idx = toAddress(x, y, z);
        occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        distance_buffer_all_[idx] = 10000;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
        int idx = toAddress(x, y, z);
        occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        distance_buffer_all_[idx] = 10000;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

      for (int y = min_cut_m(1); y < min_cut(1); ++y) {
        int idx = toAddress(x, y, z);
        occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        distance_buffer_all_[idx] = 10000;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
        int idx = toAddress(x, y, z);
        occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        distance_buffer_all_[idx] = 10000;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

      for (int x = min_cut_m(0); x < min_cut(0); ++x) {
        int idx = toAddress(x, y, z);
        occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        distance_buffer_all_[idx] = 10000;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
        int idx = toAddress(x, y, z);
        occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        distance_buffer_all_[idx] = 10000;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = local_bound_min_(0); x <= local_bound_max_(0); ++x)
    for (int y = local_bound_min_(1); y <= local_bound_max_(1); ++y)
      for (int z = local_bound_min_(2); z <= local_bound_max_(2); ++z) {
        occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = local_bound_min_(0); x <= local_bound_max_(0); ++x)
    for (int y = local_bound_min_(1); y <= local_bound_max_(1); ++y)
      for (int z = local_bound_min_(2); z <= local_bound_max_(2); ++z) {

        if (occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k) {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
              continue;
            }
            occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  /*
  if (mp_.ceil_height_ > 0.0) {
    int ceil_id = floor((mp_.ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = local_bound_min_(0); x <= local_bound_max_(0); ++x)
      for (int y = local_bound_min_(1); y <= local_bound_max_(1); ++y) {
        occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
   */
}

void SDFMap::visCallback(const ros::TimerEvent& /*event*/) {
  publishMap();
  //publishMapInflate(true);
  // publishUpdateRange();
  publishESDF();

  // publishUnknown();
  // publishDepth();
}

void SDFMap::updateOccupancyCallback(const ros::TimerEvent& /*event*/) {
  if (!occ_need_update_) return;

  /* update occupancy */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  projectDepthImage();
  raycastProcess();

  if (need_clear_local_map_) {
    clearAndInflateLocalMap();
  }

  t2 = ros::Time::now();

  fuse_time_ += (t2 - t1).toSec();
  max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());

  if (mp_.show_occ_time_)
    ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             fuse_time_ / update_num_, max_fuse_time_);

  occ_need_update_ = false;
  if (need_clear_local_map_) esdf_need_update_ = true;
  need_clear_local_map_ = false;
}

void SDFMap::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!esdf_need_update_) return;

  /* esdf */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  updateESDF3d();

  t2 = ros::Time::now();

  esdf_time_ += (t2 - t1).toSec();
  max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());

  if (mp_.show_esdf_time_)
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             esdf_time_ / update_num_, max_esdf_time_);

  esdf_need_update_ = false;
}

void SDFMap::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(depth_image_);

  // std::cout << "depth: " << depth_image_.cols << ", " << depth_image_.rows << std::endl;

  /* get pose */
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                     pose->pose.orientation.y, pose->pose.orientation.z);
  if (isInMap(camera_pos_)) {
    has_odom_ = true;
    update_num_ += 1;
    occ_need_update_ = true;
  } else {
    occ_need_update_ = false;
  }
}

void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
  if (has_first_depth_) return;

  camera_pos_(0) = odom->pose.pose.position.x;
  camera_pos_(1) = odom->pose.pose.position.y;
  camera_pos_(2) = odom->pose.pose.position.z;

  has_odom_ = true;
}

void SDFMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img) {

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  has_cloud_ = true;

  if (!has_odom_) {
    // std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0) return;

  if (isnan(camera_pos_(0)) || isnan(camera_pos_(1)) || isnan(camera_pos_(2))) return;

  this->resetBuffer(camera_pos_ - mp_.local_update_range_,
                    camera_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2)) {

      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z) {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt)) continue;

            int idx_inf = toAddress(inf_pt);

            occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  min_x = min(min_x, camera_pos_(0));
  min_y = min(min_y, camera_pos_(1));
  min_z = min(min_z, camera_pos_(2));

  max_x = max(max_x, camera_pos_(0));
  max_y = max(max_y, camera_pos_(1));
  max_z = max(max_z, camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), local_bound_min_);

  boundIndex(local_bound_min_);
  boundIndex(local_bound_max_);

  esdf_need_update_ = true;
}

void SDFMap::publishMap() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        if (occupancy_buffer_[toAddress(x, y, z)] <= mp_.min_occupancy_log_) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.points.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

void SDFMap::publishMapInflate(bool all_info) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = local_bound_min_;
  Eigen::Vector3i max_cut = local_bound_max_;

  if (all_info) {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        if (occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

void SDFMap::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = local_bound_min_;
  Eigen::Vector3i max_cut = local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        if (occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  // auto sz = max_cut - min_cut;
  // std::cout << "unknown ratio: " << cloud.width << "/" << sz(0) * sz(1) * sz(2) << "="
  //           << double(cloud.width) / (sz(0) * sz(1) * sz(2)) << std::endl;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void SDFMap::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < proj_points_cnt; ++i) {
    pt.x = proj_points_[i][0];
    pt.y = proj_points_[i][1];
    pt.z = proj_points_[i][2];
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}

void SDFMap::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  indexToPos(local_bound_min_, esdf_min_pos);
  indexToPos(local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void SDFMap::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  // Eigen::Vector3i min_cut = local_bound_min_ -
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  // Eigen::Vector3i max_cut = local_bound_max_ +
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i min_cut = local_bound_min_;
  Eigen::Vector3i max_cut = local_bound_max_;
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, -0.5), pos);
      pos(2) = mp_.esdf_slice_height_;

      dist = getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void SDFMap::getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                          vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad, int sign) {
  double dist;
  Eigen::Vector3d gd;
  for (double x = range(0); x <= range(1); x += res)
    for (double y = range(2); y <= range(3); y += res) {

      dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
      slice.push_back(Eigen::Vector3d(x, y, dist));
      grad.push_back(gd);
    }
}

void SDFMap::checkDist() {
  for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        Eigen::Vector3d grad;
        double dist = getDistWithGradTrilinear(pos, grad);

        if (fabs(dist) > 10.0) {
        }
      }
}

bool SDFMap::odomValid() {
  return has_odom_;
}

bool SDFMap::hasDepthObservation() {
  return has_first_depth_;
}

double SDFMap::getResolution() {
  return mp_.resolution_;
}

Eigen::Vector3d SDFMap::getOrigin() {
  return mp_.map_origin_;
}

int SDFMap::getVoxelNum() {
  return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void SDFMap::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2],
                            Eigen::Vector3d& diff) {
  if (!isInMap(pos)) {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y][z] = current_pos;
      }
    }
  }
}

void SDFMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& img,
                               const nav_msgs::OdometryConstPtr& odom) {
  /* get pose */
  // camera_pos_(0) = odom->pose.pose.position.x;
  // camera_pos_(1) = odom->pose.pose.position.y;
  // camera_pos_(2) = odom->pose.pose.position.z;
  // camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
  //                                    odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * cam2body_;
  camera_pos_(0) = cam_T(0, 3);
  camera_pos_(1) = cam_T(1, 3);
  camera_pos_(2) = cam_T(2, 3);
  Eigen::Matrix3d camera_r_m = cam_T.block<3, 3>(0, 0);
  camera_q_ = Eigen::Quaterniond(camera_r_m);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(depth_image_);

  occ_need_update_ = true;
}

void SDFMap::depthCallback(const sensor_msgs::ImageConstPtr& img) {
  // std::cout << "depth: " << img->header.stamp << std::endl;
}

void SDFMap::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
  // std::cout << "pose: " << pose->header.stamp << std::endl;

  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
}

// SDFMap