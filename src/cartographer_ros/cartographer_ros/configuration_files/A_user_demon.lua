-- rosservice call /finish_trajectory 0
-- rosservice call /write_state "{filename: '${HOME}/slam_map/c-map22-3-2-1804.pbstream'}"
-- rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/slam_map/c-map22-3-2-1804 -pbstream_filename=${HOME}/slam_map/c-map22-3-2-1804.pbstream -resolution=0.05

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",-- map����������ϵ�����ڷ���submap
  -- ��ֻʹ���״���н�ͼ����tracking_frame��published_frame����Ϊ�״�����ϵlaser
  -- ��ʹ���״�ͻ����˽��н�ͼ����ʹ����ʽ��̼ƣ�
  -- ��tracking_frame��published_frame����Ϊbase_link��provide_odom_frame��Ϊtrue
  -- ��ʹ���״�ͻ����˽��н�ͼ����ʹ����ʽ��̼ƣ�
  -- ��tracking_frame��Ϊbase_link��published_frame��Ϊodom��provide_odom_frame��Ϊfalse
  -- ����һ�£�odom_frame��cartographer�ü������ݼ����������̼����ݣ�
  -- provide_odom_frame��˼Ϊ�Ƿ���ҪcartographerΪ���ṩ��̼ƣ�ע��˴���̼�ָ���Ǽ�����̼�����
  -- �����provide_odom_frame����Ϊfalse����ôtf��ϵΪmap_frame->published_frame��
  -- ���published_frame��ʱΪbase_link��������tf��ϵΪmap->base_link
  -- �������ڻ���������ʱ������tf��ϵΪodom->base_link(����P3)������tf�Ĺ���һ���ڵ�ֻ����һ�����ڵ㣬���ջᵼ�±���
  -- ����������ǽ�published_frame����Ϊodom.
  tracking_frame = "base_link",--SLAM�㷨Ҫ���ٺ�У׼������ϵ�������imu��Ӧ������Ϊimu_link
  published_frame = "base_link", --һ������������̬���Ӽ�����ϵ����Ҫ����У��tracking_frame��
  odom_frame = "odom",  --����provide_odom_frame=trueʱ����ʹ�ã�λ�ù�ϵ��map_frame->odom_frame->published_frame�������������ֲ���slam������̼ƽ���������ڻػ����
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,  --���Ϊtrue���򷢲�����̬published_frame�����Ǵ�2D��̬��û�к����������ƫת�ǡ�
  use_pose_extrapolator = false,
  use_odometry = false, --���Ϊtrue�����ᶩ�ġ�odom��������⣬��Ϣ������nav_msgs/Odometry����������£������ṩ��̼����ݣ�������Щ���ݽ��������slam
  use_nav_sat = false,  --���Ϊtrue�����ᶩ�ġ�fix��������⣬��Ϣ������sensor_msgs/NavSatFix����������±����ṩ�������ݣ�������Щ���ݽ��������ȫ��slam
  use_landmarks = false,  --�Ƿ�ʹ��landmark
  num_laser_scans = 1,  --���ĵ��߼����״ﻰ�������
  num_multi_echo_laser_scans = 0, --���Ķ��߼����״ﻰ�������
  num_subdivisions_per_laser_scan = 10,  --�յ���ÿ�����߼����״����ݷָ���ĵ��Ƶ���������ɨ������ݽ���ϸ��ʹ���״����ƶ�ʱ��õ����ݲ��ᷢ�����䣬��һ�ֱ������������״��˶�����ķ�����
  num_point_clouds = 0, --���ĵĵ��ƻ��������
  lookup_transform_timeout_sec = 0.2, --ʹ��tf2��ѯ����任ʱ�ĳ�ʱʱ�䣬����Ϊ��λ
  submap_publish_period_sec = 0.3,  --������ͼ��̬��ʱ����������Ϊ��λ
  pose_publish_period_sec = 5e-3, --������̬��ʱ����������Ϊ��λ��5e-3����5����һ�Σ�200hz��һ��200��
  trajectory_publish_period_sec = 30e-3,  --�����켣��ǵ�ʱ����������Ϊ��λ
  rangefinder_sampling_ratio = 1.,  --����ǵĹ̶���������
  odometry_sampling_ratio = 1.0, --��̼ƵĹ̶�����������������ͬ
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true  --����ʹ��2D�״ﻹ��3D�״�
MAP_BUILDER.use_trajectory_builder_3d = false
--TRAJECTORY_BUILDER_2D��ǰ�������ļ�
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30 --submaps�������֡������
TRAJECTORY_BUILDER_2D.num_accumulated_range_data= 10
TRAJECTORY_BUILDER_2D.min_range = 0.2 --�����״������̾��룬��λӦ����m
TRAJECTORY_BUILDER_2D.max_range = 25  --�����״��������� ��
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25  --������������ĵ㣬���ᱻ���뵽������ȵ�λ��
TRAJECTORY_BUILDER_2D.use_imu_data = true  --�Ƿ�ʹ��imu���ݣ�3Dslam��ѡimu����
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true --�Ƿ���ʹ�������ɨ��ƥ��(csm)�������ɨ��ƥ�䣬�Ӷ�����һ���õĳ�ʼ������Ż�
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 --csm��С�������������ڣ���������ڽ����ҵ���ѵ�ɨ�����
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1  --����֡��ƥ��÷�ʱ��ƽ�ƺ���ת�������ݵ�Ȩ�أ�Ȩ��Խ�ߴ���Խ���Ÿ�����
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1  --ͬ��

--POSE_GRAPH�Ǻ�������ļ�
POSE_GRAPH.optimization_problem.huber_scale = 1e-2 --�в�������һ����������ֵԽ���쳣ֵӰ��Խ��
POSE_GRAPH.optimize_every_n_nodes = 60  --ÿ����һ��node������Ӷ�Ӧ��Լ����node����scan match��Լ���������λ�˹�ϵ���ȴ�n��node������ͻ����ȫ�ֻػ�����Ż�
POSE_GRAPH.constraint_builder.min_score = 0.55  --fast csm����С���������ڴ˷����Ż�����Ż�
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window=15
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth=7
POSE_GRAPH.optimization_problem.odometry_translation_weight=0
POSE_GRAPH.optimization_problem.odometry_rotation_weight=0

return options

