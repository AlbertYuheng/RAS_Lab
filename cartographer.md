**Cartographer** **安装及在机器人上进行建图和定位**

# 1、下载资料

链接: https://pan.baidu.com/s/144ItzZyljRe5JkJ72NNvpA  密码: dv8h

--来自百度网盘超级会员V4的分享

里面有五个文件夹abseil-cpp(谷歌开源的一个c++基础库)

​                                cartographer(cartographer源码)

​                                cartographer_ros(供我们使用的功能包)

​                      ceres-solver(谷歌开源的一个专门用来求解非线性优化问题的库)

上面四个文件夹是后续会进行安装的，cartographer_detailed是一份带有注释的cartographer源码，有兴趣的可以看一下，网址https://github.com/xiangli0608/cartographer_detailed_comments_ws，后续可能还会有更新。

# 2、安装依赖项

`（``1``）`sudo apt-get update

（2）sudo apt-get install -y clang cmake g++ git google-mock `libgmock-dev`` `libboost-all-dev libcairo2-dev libcurl4-openssl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev libsuitesparse-dev lsb-release ninja-build stow

```
libgmock-dev``这个依赖项是``ubuntu20.04``才需要安装的
```

# 3、解压后将abseil-cpp文件夹复制到主目录下

（1）cd abseil-cpp  

（2）cd build  （没有build就新建build文件）

（3）cmake -G Ninja -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/usr/local/stow/absl ..  

（4）ninja 

（5）sudo ninja install  

（6）cd /usr/local/stow  

（7）sudo stow absl



# **4****、回到主目录下运行下面的指令。**

```
注意：进行此步骤时你必须已经成功安装``ROS``。
```

注意此步骤ubuntu20.04和ubuntu18.04有区别

如果下载太慢https://blog.csdn.net/qq_52382976/article/details/121281285?spm=1001.2101.3001.6650.2&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-121281285-blog-103874277.pc_relevant_aa&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-121281285-blog-103874277.pc_relevant_aa&utm_relevant_index=5

需要文件就是rosdistro

ubuntu20.04:

（1）sudo apt-get update

（2）`sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow`

```
ubuntu18.04``及以下：
（``1``）``sudo apt-get update
（``2``）``sudo apt-get install -y python-wstool python-rosdep ninja-build stow
```

# **5****、创建一个新的工作空间****cartographer_ws**

```
因为此工作空间使用的单独的编译指令，故一定是单独的新的工作空间。
（``1``）``mkdir cartographer_ws
（``2``）``cd cartographer_ws
（``3``）``mkdir src
（``4``）将解压后的文件夹里面的``cartographer``、``cartographer_ros``、``ceres-solver``这三个文件夹放入``src``目录下``,``然后回到``cartographer_ws``目录下
（``5``）``rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
（``6``）``catkin_make_isolated --install --use-ninja
（``7``）``source install_isolated/setup.bash
```

# `6``、运行下面的指令，它会在主目录下新建``Downloads``文件夹，下载一个``2D``的``bag``包，启动``cartographer``功能包运行这个``bag``包，若能成功运行，``rviz``里面出现图像，即为成功。`

```
（``1``）``wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag 
（``2``）``roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
7``、下面的指令是一个``3D``的``bag``包，有兴趣的可以下载跑一下，这个包``9.1G
（``1``）``wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag
（``2``）``roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag
```

















































































**`在`****`p3`****`上`` ``运行`****`cartographer`****`进行建图和定位`**

```
	cartographer_ros``这个功能包只是``google``封装好的供用户调参使用的接口，不涉及``slam``过程中的算法，只是将各种传感器的数据从``ROS``的消息格式转换为``cartographer``所需要的消息格式，并传递给``cartographer``的底层源码去实现。所以我们只需要修改``cartographer_ros``这个功能包里面的一些配置文件就可以实现建图和定位的功能。 在``/home/xxx/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files``目录下是一些``cartographer``的配置文件，后缀是``.lua``，我们在使用时主要修改``.lua``文件和下面提到的``launch``文件。新建一个``my_carto.lua``文件，将下面的内容复制进去。注意在使用``cartographer``的过程中最好自己新建文件进行修改，不改变原来配置文件的内容，防止出现问题后无法修复。
注意前三个带有``--``的指令是``cartographer``保存地图的指令
rosservice call /finish_trajectory 0 ``意思是不再接收数据
```

rosservice call /write_state "{filename: '${HOME}/slam_map/c-map12.02.1.pbstream'}"  意思是序列化保存数据到你指定路径下，保存地图的路径和名字请自行修改。

rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/slam_map/c-map12.02.1 -pbstream_filename=${HOME}/slam_map/c-map12.02.1.pbstream -resolution=0.05  意思是将从你保存的路径中将pbstream文件转换为pgm和yaml，注意修改路径和名字

-------------------------------------分割线，下面是要复制的内容---------------------------------------------

-- rosservice call /finish_trajectory 0

-- rosservice call /write_state "{filename: '${HOME}/slam_map/c-map22-3-2-1804.pbstream'}"

-- rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/slam_map/c-map22-3-2-1804 -pbstream_filename=${HOME}/slam_map/c-map22-3-2-1804.pbstream -resolution=0.05



include "map_builder.lua"

include "trajectory_builder.lua"



options = {

  map_builder = MAP_BUILDER,

  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",-- map是世界坐标系，用于发布submap

  -- 若只使用雷达进行建图，则将tracking_frame和published_frame都改为雷达坐标系laser

  -- 若使用雷达和机器人进行建图，不使用轮式里程计，

  -- 则tracking_frame和published_frame都改为base_link，provide_odom_frame改为true

  -- 若使用雷达和机器人进行建图，且使用轮式里程计，

  -- 则tracking_frame改为base_link，published_frame改为odom，provide_odom_frame改为false

  -- 解释一下，odom_frame是cartographer用激光数据计算产生的里程计数据，

  -- provide_odom_frame意思为是否需要cartographer为你提供里程计，注意此处里程计指的是激光里程计数据

  -- 如果将provide_odom_frame设置为false，那么tf关系为map_frame->published_frame，

  -- 如果published_frame此时为base_link，理论上tf关系为map->base_link

  -- 但是由于机器人运行时发布的tf关系为odom->base_link(比如P3)，根据tf的规则一个节点只能有一个父节点，最终会导致报错。

  -- 解决方法就是将published_frame设置为odom.

  tracking_frame = "imu_a9_link",--SLAM算法要跟踪和校准的坐标系，如果有imu就应该设置为imu_link

  published_frame = "odom", --一个用来发布姿态的子级坐标系，主要用来校正tracking_frame；

  odom_frame = "odom",  --仅当provide_odom_frame=true时可以使用，位置关系是map_frame->odom_frame->published_frame，被用来发布局部的slam激光里程计结果，不用于回环检测

  provide_odom_frame = false,

  publish_frame_projected_to_2d = true,  --如果为true，则发布的姿态published_frame将会是纯2D姿态，没有横滚，俯仰，偏转角。

  use_pose_extrapolator = true,

  use_odometry = true, --如果为true，将会订阅“odom”这个话题，消息类型是nav_msgs/Odometry。这种情况下，必须提供里程计数据，并且这些数据将会包含进slam

  use_nav_sat = false,  --如果为true，将会订阅“fix”这个换题，消息类型是sensor_msgs/NavSatFix，这种情况下必须提供导航数据，并且这些数据将会包含进全局slam

  use_landmarks = false,  --是否使用landmark

  num_laser_scans = 1,  --订阅单线激光雷达话题的数量

  num_multi_echo_laser_scans = 0, --订阅多线激光雷达话题的数量

  num_subdivisions_per_laser_scan = 10,  --收到的每个多线激光雷达数据分割出的点云的数量。对扫描的数据进行细分使得雷达在移动时获得的数据不会发生畸变，是一种暴力消除激光雷达运动畸变的方法。

  num_point_clouds = 0, --订阅的点云话题的数量

  lookup_transform_timeout_sec = 0.2, --使用tf2查询坐标变换时的超时时间，以秒为单位

  submap_publish_period_sec = 0.3,  --发布子图姿态的时间间隔，以秒为单位

  pose_publish_period_sec = 5e-3, --发布姿态的时间间隔，以秒为单位；5e-3代表5毫秒一次，200hz，一秒200次

  trajectory_publish_period_sec = 30e-3,  --发布轨迹标记的时间间隔，以秒为单位

  rangefinder_sampling_ratio = 1.,  --测距仪的固定采样比例

  odometry_sampling_ratio = 1.0, --里程计的固定采样比例，以下类同

  fixed_frame_pose_sampling_ratio = 1.,

  imu_sampling_ratio = 1.,

  landmarks_sampling_ratio = 1.,

}



MAP_BUILDER.use_trajectory_builder_2d = true  --决定使用2D雷达还是3D雷达

MAP_BUILDER.use_trajectory_builder_3d = false

--TRAJECTORY_BUILDER_2D是前端配置文件

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30 --submaps插入点云帧的数量

TRAJECTORY_BUILDER_2D.num_accumulated_range_data= 10

TRAJECTORY_BUILDER_2D.min_range = 0.2 --激光雷达测距的最短距离，单位应该是m

TRAJECTORY_BUILDER_2D.max_range = 25  --激光雷达测距的最长距离 。

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25  --超过测距最长距离的点，将会被插入到这个长度的位置

TRAJECTORY_BUILDER_2D.use_imu_data = true  --是否使用imu数据，3Dslam必选imu数据

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true --是否先使用相关性扫描匹配(csm)解决在线扫描匹配，从而产生一个好的初始点进行优化

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 --csm最小的线性搜索窗口，在这个窗口将会找到最佳的扫描队列

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1  --计算帧间匹配得分时，平移和旋转部分数据的权重，权重越高代表越相信该数据

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1  --同上



--POSE_GRAPH是后端配置文件

POSE_GRAPH.optimization_problem.huber_scale = 1e-2 --残差方程里面的一个参数，数值越大，异常值影响越大

POSE_GRAPH.optimize_every_n_nodes = 60  --每结束一个node都会添加对应的约束。node代表scan match，约束代表相对位姿关系。等待n个node结束后就会进行全局回环检测优化

POSE_GRAPH.constraint_builder.min_score = 0.55  --fast csm的最小分数，高于此分数才会进行优化

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window=15

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth=7

POSE_GRAPH.optimization_problem.odometry_translation_weight=0

POSE_GRAPH.optimization_problem.odometry_rotation_weight=0



return options

------------------------------------------分割线------------------------------------------------------------------

```
在``/home/xxx/cartographer_ws/src/cartographer_ros/cartographer_ros/launch``目录下新建``my_carto.launch``文件，将下面内容复制进去。
---------------------------------------``分割线，下面是要复制的内容``--------------------------------------
<launch> 
  ``<!-- ``若为``true``，意为使用仿真；若为``false``，意为使用实物 ``-->
  ``<!-- my_carto.lua``是我们自己用的配置文件 ``-->
  ``<!-- remap from="scan" to="scan" ``是雷达发布的话题映射 ``-->
  ``<param name="/use_sim_time" value="false" />  
 
```

<!-- 下面这个节点的功能是从你的.lua配置文件中读取参数，开启SLAM，注意修改文件路径和名字 -->

```
  ``<node name="cartographer_node" pkg="cartographer_ros"  
        ``type="cartographer_node" args="  
            ``-configuration_directory $(find cartographer_ros)/configuration_files  
            ``-configuration_basename my_carto.lua"  
        ``output="screen"> 
    ``<remap from="scan" to="scan" />  
  ``</node> 
  
<!-- ``下面这个节点的功能是更新栅格地图并发布 ``-->
 ``<node pkg="cartographer_ros" type="cartographer_occupancy_grid_node"
        ``name="cartographer_occupancy_grid_node"
        ``args="-resolution 0.05" />
        
  ``<node name="rviz" pkg="rviz" type="rviz" required="true"  
        ``args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" /> 
</launch>
```

---------------------------------------------------分割线-----------------------------------------------------------

修改结束后，运行下面两条指令，之后每次修改，都要运行一下。

```
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
	``之后开启雷达和小车，再运行``my_carto.launch``文件，即可进行建图，建图完成后注意使用上面提到的三条命令进行地图的保存。在接下来的定位功能中就需要使用已经建好的地图。
```



```
	cartographer``本身就带有重定位的功能，那么如何在一张已经建好的地图上进行重定位那？``	cartographer``已经提供好了这个功能的接口。``backpack_2d_localization.lua``和``demo_backpack_2d_localization.launch``这两个文件就是定位功能的接口，我们只需要修改一些内容。
	``首先在``/home/xxx/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files``目录下新建一个``my_backpack_2d_localization.lua``文件，将下面的内容复制进去。
-----------------------------------------------``分割线，下面是要复制的内容``------------------------------------
```

include "my_carto.lua"



TRAJECTORY_BUILDER.pure_localization_trimmer = {

  max_submaps_to_keep = 3, --最大保存子图数，纯定位模式通过子图进行定位，但只需要和上一个子图

}

POSE_GRAPH.optimize_every_n_nodes = 20  --每20个有效帧一个子图，子图构建完成要回环检测一次，这个数越小回环检测越频繁，计算量越大



return options

```
-----------------------------------------------``分割线``--------------------------------------------------------------
然后在``/home/xxx/cartographer_ws/src/cartographer_ros/cartographer_ros/launch``目录下新建``my_backpack_2d_localization.launch``文件，将下面内容复制进去。
---------------------------------------``分割线，下面是要复制的内容``--------------------------------------
```

<launch>

  <param name="/use_sim_time" value="false" />



<!--configuration_directory是你的.lua配置文件的路径

​    configuration_basename是你的.lua配置文件的名字

​    load_state_filename是你保存好的地图数据的路径和名字-->

<!-- 下面这个节点的功能是从你的.lua配置文件中读取参数，开启SLAM -->

  <node name="cartographer_node" pkg="cartographer_ros"

​      type="cartographer_node" args="

​          -configuration_directory $(find cartographer_ros)/configuration_files

​          -configuration_basename my_backpack_2d_localization.lua

​          -load_state_filename /home/yuan/slam_map/c-map12.02.1.pbstream"

​      output="screen">

​    <remap from="scan" to="scan" />

  </node>



<!-- 下面这个节点的功能是更新栅格地图并发布 -->

  <!-- <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros" -->

​      <!-- type="cartographer_occupancy_grid_node" args="-resolution 0.05" /--> 

<!--下面的指令调用 map_server发布地图数据，而不使用cartographer_occupancy_grid_node这个节点

​      原因是为了避免在定位的过程中将获得的雷达数据在已经建好的地图上进行更新，使地图发生漂移，定位效果变差

​      注意修改文件名字和路径-->

​      <node name="map_server" pkg="map_server" type="map_server" args="/home/yuan/slam_map/c-map12.02.1.yaml"/>



  <node name="rviz" pkg="rviz" type="rviz" required="true"

​      args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" />



</launch>

```
------------------------------------------------``分割线``-------------------------------------------------------------
	``最后一步在``/home/xxx/cartographer_ws/src/cartographer_ros/cartographer_ros/cartographer_ros/node_options.h``文件中第``39``行将``publish_tracked_pose``的值修改为``true``。
之后在工作空间下进行编译和``source
catkin_make_isolated --install –use-ninja
source install_isolated/setup.bash
接下来用已经建好的地图运行``my_backpack_2d_localization.launch``文件，开启之后机器人会出现在你建图时最开始的位置，之后控制机器人移动，``cartographer``便会根据采集到的雷达数据不停的进行对比，在``rviz``中将机器人的位置修正到正确的地方。
在定位的过程中，``/tracked_pose``这个话题会一直输出机器人在地图坐标系下的位姿。
```



关于纯定位

https://developer.aliyun.com/article/858059

关于pose_graph参数

https://blog.csdn.net/windxf/article/details/109195442

关于pose_graph

[https://gaoyichao.com/Xiaotu/?book=Cartographer%E6%BA%90%E7%A0%81%E8%A7%A3%E8%AF%BB&title=Global_SLAM%E7%9A%84%E6%A0%B8%E5%BF%83_PoseGraph2D](https://gaoyichao.com/Xiaotu/?book=Cartographer源码解读&title=Global_SLAM的核心_PoseGraph2D)

关于amcl

https://blog.csdn.net/Numberors/article/details/114652824?spm=1001.2014.3001.5501

关于dwa

https://zhuanlan.zhihu.com/p/481962661



https://blog.csdn.net/xi_shui/article/details/119006254



cartographer_mir-master 与landmark有关



https://google-cartographer-ros.readthedocs.io/en/latest/demos.html#



修改POSE_GRAPH.optimization_problem.huber_scale = 5e-2回环频率增加，建图效果提高

修改TRAJECTORY_BUILDER_2D.max_range = 15 --激光雷达测距的最长距离 。25

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 15 --超过测距最长距离的点，将会被插入到这个长度的位置 25

限制雷达激光范围，减小误差







```
若在使用过程中出现问题，请联系我进行补充修改。
```