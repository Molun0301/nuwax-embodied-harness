[env.sh] ROS_DISTRO=foxy  RMW=rmw_cyclonedds_cpp  CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
[go2.sh] Cleaning up any leftover Nav2/RTAB-Map/explorer processes...
[go2.sh] MODE = exploration (autonomous mapping)
[go2.sh]   database_path = /home/unitree/rtabmap.db  (will be overwritten)
[go2.sh]   nav2_log_only = true (non-Nav2 node output -> ~/.ros/log)

[INFO] [launch]: All log files can be found below /home/unitree/.ros/log/2026-06-04-15-38-56-903873-ubuntu-152442
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [odom_tf_broadcaster.py-1]: process started with pid [152632]
[INFO] [cloud_to_scan.py-2]: process started with pid [152634]
[INFO] [rtabmap-3]: process started with pid [152636]
[INFO] [cmd_vel_bridge.py-4]: process started with pid [152638]
[cmd_vel_bridge.py-4] [INFO] [1780558738.894971395] [cmd_vel_bridge]: CmdVel bridge started. Limits: vx=0.4, vy=0.2, vyaw=0.8
[odom_tf_broadcaster.py-1] [INFO] [1780558739.030317934] [odom_tf_broadcaster]: OdomTfBroadcaster started: /utlidar/robot_odom -> TF odom->base_link
[cloud_to_scan.py-2] [INFO] [1780558739.700427548] [cloud_to_scan]: CloudToScan: /utlidar/cloud_deskewed -> /scan (frame=base_link, z=-0.20..0.30, r=0.30..8.00)
[rtabmap-3] [INFO] [1780558740.742473368] [rtabmap]: rtabmap(maps): map_filter_radius          = 0.000000
[rtabmap-3] [INFO] [1780558740.745625458] [rtabmap]: rtabmap(maps): map_filter_angle           = 30.000000
[rtabmap-3] [INFO] [1780558740.748750828] [rtabmap]: rtabmap(maps): map_cleanup                = true
[rtabmap-3] [INFO] [1780558740.748863950] [rtabmap]: rtabmap(maps): map_always_update          = false
[rtabmap-3] [INFO] [1780558740.748892911] [rtabmap]: rtabmap(maps): map_empty_ray_tracing      = true
[rtabmap-3] [INFO] [1780558740.748915439] [rtabmap]: rtabmap(maps): cloud_output_voxelized     = true
[rtabmap-3] [INFO] [1780558740.748935376] [rtabmap]: rtabmap(maps): cloud_subtract_filtering   = false
[rtabmap-3] [INFO] [1780558740.748953616] [rtabmap]: rtabmap(maps): cloud_subtract_filtering_min_neighbors = 2
[rtabmap-3] [INFO] [1780558740.749108691] [rtabmap]: rtabmap(maps): octomap_tree_depth         = 16
[rtabmap-3] [INFO] [1780558741.017949304] [rtabmap]: rtabmap: frame_id      = base_link
[rtabmap-3] [INFO] [1780558741.021342743] [rtabmap]: rtabmap: odom_frame_id = odom
[rtabmap-3] [INFO] [1780558741.022841075] [rtabmap]: rtabmap: map_frame_id  = map
[rtabmap-3] [INFO] [1780558741.024787415] [rtabmap]: rtabmap: log_to_rosout_level  = 4
[rtabmap-3] [INFO] [1780558741.027617484] [rtabmap]: rtabmap: initial_pose  = 
[rtabmap-3] [INFO] [1780558741.027668013] [rtabmap]: rtabmap: use_action_for_goal  = false
[rtabmap-3] [INFO] [1780558741.027690957] [rtabmap]: rtabmap: tf_delay      = 0.050000
[rtabmap-3] [INFO] [1780558741.027720174] [rtabmap]: rtabmap: tf_tolerance  = 1.000000
[rtabmap-3] [INFO] [1780558741.027740398] [rtabmap]: rtabmap: odom_sensor_sync   = false
[rtabmap-3] [INFO] [1780558741.027758766] [rtabmap]: rtabmap: gen_scan  = false
[rtabmap-3] [INFO] [1780558741.027776911] [rtabmap]: rtabmap: gen_depth  = false
[rtabmap-3] [INFO] [1780558741.027816015] [rtabmap]: rtabmap: scan_cloud_max_points = 0
[rtabmap-3] [INFO] [1780558741.027835440] [rtabmap]: rtabmap: scan_cloud_is_2d = false
[rtabmap-3] [INFO] [1780558741.323771158] [rtabmap]: Setting RTAB-Map parameter "Grid/CellSize"="0.10"
[rtabmap-3] [INFO] [1780558741.333728943] [rtabmap]: Setting RTAB-Map parameter "Grid/MaxGroundHeight"="-0.15"
[rtabmap-3] [INFO] [1780558741.334302714] [rtabmap]: Setting RTAB-Map parameter "Grid/MaxObstacleHeight"="1.0"
[rtabmap-3] [INFO] [1780558741.334729122] [rtabmap]: Setting RTAB-Map parameter "Grid/MinClusterSize"="3"
[rtabmap-3] [INFO] [1780558741.335133994] [rtabmap]: Setting RTAB-Map parameter "Grid/MinGroundHeight"="-0.50"
[rtabmap-3] [INFO] [1780558741.335541233] [rtabmap]: Setting RTAB-Map parameter "Grid/NoiseFilteringMinNeighbors"="0"
[rtabmap-3] [INFO] [1780558741.337197584] [rtabmap]: Setting RTAB-Map parameter "Grid/NormalsSegmentation"="false"
[rtabmap-3] [INFO] [1780558741.338088737] [rtabmap]: Setting RTAB-Map parameter "Grid/RangeMax"="8.0"
[rtabmap-3] [INFO] [1780558741.338520713] [rtabmap]: Setting RTAB-Map parameter "Grid/RangeMin"="0.3"
[rtabmap-3] [INFO] [1780558741.338925584] [rtabmap]: Setting RTAB-Map parameter "Grid/RayTracing"="true"
[rtabmap-3] [INFO] [1780558741.344840446] [rtabmap]: Setting RTAB-Map parameter "Grid/Sensor"="0"
[rtabmap-3] [INFO] [1780558741.345559180] [rtabmap]: Setting RTAB-Map parameter "GridGlobal/ProbClampingMax"="0.95"
[rtabmap-3] [INFO] [1780558741.345769008] [rtabmap]: Setting RTAB-Map parameter "GridGlobal/ProbClampingMin"="0.10"
[rtabmap-3] [INFO] [1780558741.345955059] [rtabmap]: Setting RTAB-Map parameter "GridGlobal/ProbHit"="0.85"
[rtabmap-3] [INFO] [1780558741.346190647] [rtabmap]: Setting RTAB-Map parameter "GridGlobal/ProbMiss"="0.45"
[rtabmap-3] [INFO] [1780558741.348570404] [rtabmap]: Setting RTAB-Map parameter "Icp/Iterations"="20"
[rtabmap-3] [INFO] [1780558741.348784104] [rtabmap]: Setting RTAB-Map parameter "Icp/MaxCorrespondenceDistance"="0.2"
[rtabmap-3] [INFO] [1780558741.352278217] [rtabmap]: Setting RTAB-Map parameter "Icp/VoxelSize"="0.1"
[rtabmap-3] [INFO] [1780558741.375906657] [rtabmap]: Setting RTAB-Map parameter "Mem/InitWMWithAllNodes"="true"
[rtabmap-3] [INFO] [1780558741.376931508] [rtabmap]: Setting RTAB-Map parameter "Mem/NotLinkedNodesKept"="false"
[rtabmap-3] [INFO] [1780558741.377975815] [rtabmap]: Setting RTAB-Map parameter "Mem/STMSize"="30"
[rtabmap-3] [INFO] [1780558741.380445141] [rtabmap]: Setting RTAB-Map parameter "RGBD/AngularUpdate"="0.05"
[rtabmap-3] [INFO] [1780558741.381006912] [rtabmap]: Setting RTAB-Map parameter "RGBD/LinearUpdate"="0.05"
[rtabmap-3] [INFO] [1780558741.381826351] [rtabmap]: Setting RTAB-Map parameter "RGBD/NeighborLinkRefining"="true"
[rtabmap-3] [INFO] [1780558741.383062982] [rtabmap]: Setting RTAB-Map parameter "RGBD/ProximityPathMaxNeighbors"="10"
[rtabmap-3] [INFO] [1780558741.383378220] [rtabmap]: Setting RTAB-Map parameter "Reg/Force3DoF"="true"
[rtabmap-3] [INFO] [1780558741.383536879] [rtabmap]: Setting RTAB-Map parameter "Reg/Strategy"="1"
[rtabmap-3] [INFO] [1780558741.383700946] [rtabmap]: Setting RTAB-Map parameter "Rtabmap/CreateIntermediateNodes"="true"
[rtabmap-3] [INFO] [1780558741.383803636] [rtabmap]: Setting RTAB-Map parameter "Rtabmap/DetectionRate"="3.0"
[rtabmap-3] [INFO] [1780558741.422087485] [rtabmap]: Setting "Icp/PointToPlaneRadius" parameter to 0 (default 0.000000) as "subscribe_scan_cloud" is true.
[rtabmap-3] [INFO] [1780558741.423818621] [rtabmap]: RTAB-Map detection rate = 3.000000 Hz
[rtabmap-3] [INFO] [1780558741.423961855] [rtabmap]: Create intermediate nodes
[rtabmap-3] [INFO] [1780558741.429879150] [rtabmap]: rtabmap: Deleted database "/home/unitree/rtabmap.db" (--delete_db_on_start or -d are set).
[rtabmap-3] [INFO] [1780558741.430082641] [rtabmap]: rtabmap: Using database from "/home/unitree/rtabmap.db" (0 MB).
[rtabmap-3] [INFO] [1780558741.598125834] [rtabmap]: rtabmap: Database version = "0.21.1".
[rtabmap-3] [INFO] [1780558741.603192937] [rtabmap]: rtabmap: SLAM mode (Mem/IncrementalMemory=true)
[rtabmap-3] [INFO] [1780558741.781297437] [rtabmap]: Setup callbacks
[rtabmap-3] [INFO] [1780558741.781809382] [rtabmap]: rtabmap: subscribe_depth = false
[rtabmap-3] [INFO] [1780558741.781870119] [rtabmap]: rtabmap: subscribe_rgb = false
[rtabmap-3] [INFO] [1780558741.781897576] [rtabmap]: rtabmap: subscribe_stereo = false
[rtabmap-3] [INFO] [1780558741.781917576] [rtabmap]: rtabmap: subscribe_rgbd = false (rgbd_cameras=1)
[rtabmap-3] [INFO] [1780558741.781938697] [rtabmap]: rtabmap: subscribe_odom_info = false
[rtabmap-3] [INFO] [1780558741.781957833] [rtabmap]: rtabmap: subscribe_user_data = false
[rtabmap-3] [INFO] [1780558741.781981353] [rtabmap]: rtabmap: subscribe_scan = false
[rtabmap-3] [INFO] [1780558741.782000938] [rtabmap]: rtabmap: subscribe_scan_cloud = true
[rtabmap-3] [INFO] [1780558741.782018858] [rtabmap]: rtabmap: subscribe_scan_descriptor = false
[rtabmap-3] [INFO] [1780558741.782037194] [rtabmap]: rtabmap: queue_size      = 10
[rtabmap-3] [INFO] [1780558741.782063467] [rtabmap]: rtabmap: qos_image       = 0
[rtabmap-3] [INFO] [1780558741.782083819] [rtabmap]: rtabmap: qos_camera_info = 0
[rtabmap-3] [INFO] [1780558741.782101292] [rtabmap]: rtabmap: qos_scan        = 0
[rtabmap-3] [INFO] [1780558741.782119020] [rtabmap]: rtabmap: qos_odom        = 1
[rtabmap-3] [INFO] [1780558741.782136492] [rtabmap]: rtabmap: qos_user_data   = 0
[rtabmap-3] [INFO] [1780558741.782154285] [rtabmap]: rtabmap: approx_sync     = true
[rtabmap-3] [INFO] [1780558741.782604469] [rtabmap]: Setup scan callback
[rtabmap-3] [INFO] [1780558741.785278407] [rtabmap]: 
[rtabmap-3] rtabmap subscribed to:
[rtabmap-3]    /utlidar/cloud_deskewed
[rtabmap-3] [WARN] [1780558741.786706465] [rtabmap]: There is no image subscription, bag-of-words loop closure detection will be disabled...
[rtabmap-3] [WARN] [1780558741.786769091] [rtabmap]: Setting Kp/MaxFeatures=-1 (bag-of-words disabled)
[rtabmap-3] [INFO] [1780558742.019522577] [rtabmap]: rtabmap (1): Rate=0.33s, Limit=0.000s, Conversion=0.0802s, RTAB-Map=0.0596s, Maps update=0.0008s pub=0.0010s (local map=1, WM=1)
[rtabmap-3] [WARN] [1780558742.022273924] [rtabmap]: New subscription discovered on this topic, requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: DURABILITY_QOS_POLICY
[rtabmap-3] [INFO] [1780558742.137979219] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558742.138143254] [rtabmap]: rtabmap (2): Rate=0.33s, Limit=0.000s, Conversion=0.1099s, RTAB-Map=0.0052s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558742.170654548] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558742.170815383] [rtabmap]: rtabmap (3): Rate=0.33s, Limit=0.000s, Conversion=0.0305s, RTAB-Map=0.0016s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558742.223652113] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558742.223850676] [rtabmap]: rtabmap (4): Rate=0.33s, Limit=0.000s, Conversion=0.0262s, RTAB-Map=0.0176s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558742.457113940] [rtabmap]: rtabmap (5): Rate=0.33s, Limit=0.000s, Conversion=0.0266s, RTAB-Map=0.1884s, Maps update=0.0005s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558742.483656419] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558742.483858374] [rtabmap]: rtabmap (6): Rate=0.33s, Limit=0.000s, Conversion=0.0242s, RTAB-Map=0.0016s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558742.557679304] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558742.557862795] [rtabmap]: rtabmap (7): Rate=0.33s, Limit=0.000s, Conversion=0.0435s, RTAB-Map=0.0080s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558742.717574286] [rtabmap]: rtabmap (8): Rate=0.33s, Limit=0.000s, Conversion=0.0009s, RTAB-Map=0.0715s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558742.766336732] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558742.766518399] [rtabmap]: rtabmap (9): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0479s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558742.782385831] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558742.782547562] [rtabmap]: rtabmap (10): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558742.956074575] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558742.958008467] [rtabmap]: rtabmap (11): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0626s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558743.021887291] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558743.022112032] [rtabmap]: rtabmap (12): Rate=0.33s, Limit=0.000s, Conversion=0.0593s, RTAB-Map=0.0028s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558743.154793071] [rtabmap]: rtabmap (13): Rate=0.33s, Limit=0.000s, Conversion=0.0774s, RTAB-Map=0.0420s, Maps update=0.0005s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558743.221200968] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558743.221392972] [rtabmap]: rtabmap (14): Rate=0.33s, Limit=0.000s, Conversion=0.0640s, RTAB-Map=0.0017s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558743.423428620] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558743.423608303] [rtabmap]: rtabmap (15): Rate=0.33s, Limit=0.000s, Conversion=0.1628s, RTAB-Map=0.0210s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[INFO] [controller_server-5]: process started with pid [152812]
[INFO] [planner_server-6]: process started with pid [152814]
[INFO] [recoveries_server-7]: process started with pid [152816]
[INFO] [bt_navigator-8]: process started with pid [152818]
[INFO] [lifecycle_manager-9]: process started with pid [152820]
[rtabmap-3] [INFO] [1780558743.896757625] [rtabmap]: rtabmap (16): Rate=0.33s, Limit=0.000s, Conversion=0.1651s, RTAB-Map=0.3071s, Maps update=0.0003s pub=0.0001s (local map=1, WM=1)
[planner_server-6] [INFO] [1780558743.957608971] [planner_server]: 
[planner_server-6] 	planner_server lifecycle node launched. 
[planner_server-6] 	Waiting on external lifecycle transitions to activate
[planner_server-6] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[planner_server-6] [INFO] [1780558744.000624654] [planner_server]: Creating
[lifecycle_manager-9] [INFO] [1780558744.027511942] [lifecycle_manager_navigation]: Creating
[planner_server-6] [INFO] [1780558744.045597016] [global_costmap.global_costmap]: 
[planner_server-6] 	global_costmap lifecycle node launched. 
[planner_server-6] 	Waiting on external lifecycle transitions to activate
[planner_server-6] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[planner_server-6] [INFO] [1780558744.049322014] [global_costmap.global_costmap]: Creating Costmap
[rtabmap-3] [INFO] [1780558744.070459754] [rtabmap]: rtabmap (17): Rate=0.33s, Limit=0.000s, Conversion=0.1391s, RTAB-Map=0.0335s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558744.131914216] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558744.132116428] [rtabmap]: rtabmap (18): Rate=0.33s, Limit=0.000s, Conversion=0.0554s, RTAB-Map=0.0054s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558744.241754225] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558744.242134840] [rtabmap]: rtabmap (19): Rate=0.33s, Limit=0.000s, Conversion=0.1049s, RTAB-Map=0.0031s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558744.267736663] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558744.267899963] [rtabmap]: rtabmap (20): Rate=0.33s, Limit=0.000s, Conversion=0.0210s, RTAB-Map=0.0034s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[lifecycle_manager-9] [INFO] [1780558744.409244657] [lifecycle_manager_navigation]: [34m[1mCreating and initializing lifecycle service clients[0m[0m
[controller_server-5] [INFO] [1780558744.426477332] [controller_server]: 
[controller_server-5] 	controller_server lifecycle node launched. 
[controller_server-5] 	Waiting on external lifecycle transitions to activate
[controller_server-5] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[controller_server-5] [INFO] [1780558744.436922455] [controller_server]: Creating controller server
[rtabmap-3] [INFO] [1780558744.459450237] [rtabmap]: rtabmap (21): Rate=0.33s, Limit=0.000s, Conversion=0.0604s, RTAB-Map=0.1302s, Maps update=0.0003s pub=0.0001s (local map=1, WM=1)
[lifecycle_manager-9] [INFO] [1780558744.574194209] [lifecycle_manager_navigation]: [34m[1mStarting managed nodes bringup...[0m[0m
[rtabmap-3] [INFO] [1780558744.575587163] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558744.575756158] [rtabmap]: rtabmap (22): Rate=0.33s, Limit=0.000s, Conversion=0.1127s, RTAB-Map=0.0028s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[lifecycle_manager-9] [INFO] [1780558744.583270955] [lifecycle_manager_navigation]: [34m[1mConfiguring controller_server[0m[0m
[controller_server-5] [INFO] [1780558744.658464747] [local_costmap.local_costmap]: 
[controller_server-5] 	local_costmap lifecycle node launched. 
[controller_server-5] 	Waiting on external lifecycle transitions to activate
[controller_server-5] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[controller_server-5] [INFO] [1780558744.659394972] [local_costmap.local_costmap]: Creating Costmap
[recoveries_server-7] [INFO] [1780558744.667786010] [recovery_server]: 
[recoveries_server-7] 	recovery_server lifecycle node launched. 
[recoveries_server-7] 	Waiting on external lifecycle transitions to activate
[recoveries_server-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[bt_navigator-8] [INFO] [1780558744.891199344] [bt_navigator]: 
[bt_navigator-8] 	bt_navigator lifecycle node launched. 
[bt_navigator-8] 	Waiting on external lifecycle transitions to activate
[bt_navigator-8] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[bt_navigator-8] [INFO] [1780558744.891584504] [bt_navigator]: Creating
[rtabmap-3] [INFO] [1780558744.991272162] [rtabmap]: rtabmap (23): Rate=0.33s, Limit=0.000s, Conversion=0.1677s, RTAB-Map=0.1697s, Maps update=0.0003s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558745.094815032] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558745.096488471] [rtabmap]: rtabmap (24): Rate=0.33s, Limit=0.000s, Conversion=0.0715s, RTAB-Map=0.0315s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558745.288859248] [rtabmap]: rtabmap (25): Rate=0.33s, Limit=0.000s, Conversion=0.0410s, RTAB-Map=0.1491s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558745.391607031] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558745.391786746] [rtabmap]: rtabmap (26): Rate=0.33s, Limit=0.000s, Conversion=0.0831s, RTAB-Map=0.0190s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558745.450091072] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558745.450251075] [rtabmap]: rtabmap (27): Rate=0.33s, Limit=0.000s, Conversion=0.0427s, RTAB-Map=0.0127s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558745.628307951] [rtabmap]: rtabmap (28): Rate=0.33s, Limit=0.000s, Conversion=0.0468s, RTAB-Map=0.1297s, Maps update=0.0004s pub=0.0007s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558745.666944355] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558745.670339779] [rtabmap]: rtabmap (29): Rate=0.33s, Limit=0.000s, Conversion=0.0324s, RTAB-Map=0.0033s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558745.793149091] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558745.805546699] [rtabmap]: rtabmap (30): Rate=0.33s, Limit=0.000s, Conversion=0.0741s, RTAB-Map=0.0044s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[controller_server-5] [INFO] [1780558746.041499158] [controller_server]: Configuring controller interface
[controller_server-5] [INFO] [1780558746.043833506] [controller_server]: Controller frequency set to 5.0000Hz
[controller_server-5] [INFO] [1780558746.043943332] [local_costmap.local_costmap]: Configuring
[controller_server-5] [INFO] [1780558746.089831619] [local_costmap.local_costmap]: Using plugin "obstacle_layer"
[controller_server-5] [INFO] [1780558746.171365119] [local_costmap.local_costmap]: Subscribed to Topics: scan
[rtabmap-3] [INFO] [1780558746.209184741] [rtabmap]: rtabmap (31): Rate=0.33s, Limit=0.000s, Conversion=0.0706s, RTAB-Map=0.3073s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558746.242667963] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558746.242831582] [rtabmap]: rtabmap (32): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0327s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[controller_server-5] [INFO] [1780558746.334136850] [local_costmap.local_costmap]: Initialized plugin "obstacle_layer"
[controller_server-5] [INFO] [1780558746.334766525] [local_costmap.local_costmap]: Using plugin "inflation_layer"
[controller_server-5] [INFO] [1780558746.378685463] [local_costmap.local_costmap]: Initialized plugin "inflation_layer"
[controller_server-5] [INFO] [1780558746.568536102] [controller_server]: Created progress_checker : progress_checker of type nav2_controller::SimpleProgressChecker
[rtabmap-3] [INFO] [1780558746.569027151] [rtabmap]: rtabmap (33): Rate=0.33s, Limit=0.000s, Conversion=0.0418s, RTAB-Map=0.1854s, Maps update=0.0002s pub=0.0001s (local map=1, WM=1)
[controller_server-5] [INFO] [1780558746.598846112] [controller_server]: Created goal_checker : goal_checker of type nav2_controller::SimpleGoalChecker
[controller_server-5] [INFO] [1780558746.627070386] [controller_server]: Created controller : FollowPath of type nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
[controller_server-5] [INFO] [1780558746.697332315] [controller_server]: Controller Server has FollowPath  controllers available.
[rtabmap-3] [INFO] [1780558746.713576844] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558746.713801264] [rtabmap]: rtabmap (34): Rate=0.33s, Limit=0.000s, Conversion=0.1225s, RTAB-Map=0.0195s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[lifecycle_manager-9] [INFO] [1780558746.747557002] [lifecycle_manager_navigation]: [34m[1mConfiguring planner_server[0m[0m
[planner_server-6] [INFO] [1780558746.749301995] [planner_server]: Configuring
[planner_server-6] [INFO] [1780558746.749432878] [global_costmap.global_costmap]: Configuring
[planner_server-6] [INFO] [1780558746.817578254] [global_costmap.global_costmap]: Using plugin "static_layer"
[rtabmap-3] [INFO] [1780558746.828973796] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558746.829162536] [rtabmap]: rtabmap (35): Rate=0.33s, Limit=0.000s, Conversion=0.1100s, RTAB-Map=0.0045s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[planner_server-6] [INFO] [1780558746.915021749] [global_costmap.global_costmap]: Subscribing to the map topic (/map) with transient local durability
[planner_server-6] [INFO] [1780558746.931053699] [global_costmap.global_costmap]: Initialized plugin "static_layer"
[planner_server-6] [INFO] [1780558746.931197189] [global_costmap.global_costmap]: Using plugin "obstacle_layer"
[planner_server-6] [INFO] [1780558746.947391382] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 96 X 68 at 0.100000 m/pix
[planner_server-6] [INFO] [1780558746.965471497] [global_costmap.global_costmap]: Subscribed to Topics: scan
[planner_server-6] [INFO] [1780558747.118790926] [global_costmap.global_costmap]: Initialized plugin "obstacle_layer"
[planner_server-6] [INFO] [1780558747.122603190] [global_costmap.global_costmap]: Using plugin "inflation_layer"
[planner_server-6] [INFO] [1780558747.218575813] [global_costmap.global_costmap]: Initialized plugin "inflation_layer"
[rtabmap-3] [INFO] [1780558747.224663576] [rtabmap]: rtabmap (36): Rate=0.33s, Limit=0.000s, Conversion=0.1563s, RTAB-Map=0.2277s, Maps update=0.0108s pub=0.0001s (local map=1, WM=1)
[planner_server-6] [INFO] [1780558747.516340523] [planner_server]: Created global planner plugin GridBased of type nav2_navfn_planner/NavfnPlanner
[planner_server-6] [INFO] [1780558747.516529134] [planner_server]: Configuring plugin GridBased of type NavfnPlanner
[rtabmap-3] [INFO] [1780558747.519238177] [rtabmap]: rtabmap (37): Rate=0.33s, Limit=0.000s, Conversion=0.0411s, RTAB-Map=0.2445s, Maps update=0.0005s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558747.527374554] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558747.527548862] [rtabmap]: rtabmap (38): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0072s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558747.671124877] [rtabmap]: rtabmap (39): Rate=0.33s, Limit=0.000s, Conversion=0.0216s, RTAB-Map=0.0603s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[planner_server-6] [INFO] [1780558747.727553555] [planner_server]: Planner Server has GridBased  planners available.
[rtabmap-3] [INFO] [1780558747.732370190] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558747.732538001] [rtabmap]: rtabmap (40): Rate=0.33s, Limit=0.000s, Conversion=0.0013s, RTAB-Map=0.0151s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558747.802185936] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558747.802367571] [rtabmap]: rtabmap (41): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0191s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558747.951445098] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558747.951625389] [rtabmap]: rtabmap (42): Rate=0.33s, Limit=0.000s, Conversion=0.0855s, RTAB-Map=0.0254s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[lifecycle_manager-9] [INFO] [1780558748.067523765] [lifecycle_manager_navigation]: [34m[1mConfiguring recovery_server[0m[0m
[recoveries_server-7] [INFO] [1780558748.069099027] [recovery_server]: Configuring
[recoveries_server-7] [INFO] [1780558748.107614921] [recovery_server]: Creating recovery plugin spin of type nav2_recoveries/Spin
[recoveries_server-7] [INFO] [1780558748.114523948] [recovery_server]: Configuring spin
[recoveries_server-7] [INFO] [1780558748.160216010] [recovery_server]: Creating recovery plugin backup of type nav2_recoveries/BackUp
[recoveries_server-7] [INFO] [1780558748.165387691] [recovery_server]: Configuring backup
[rtabmap-3] [INFO] [1780558748.176054100] [rtabmap]: rtabmap (43): Rate=0.33s, Limit=0.000s, Conversion=0.1067s, RTAB-Map=0.0360s, Maps update=0.0003s pub=0.0001s (local map=1, WM=1)
[recoveries_server-7] [INFO] [1780558748.231215109] [recovery_server]: Creating recovery plugin wait of type nav2_recoveries/Wait
[recoveries_server-7] [INFO] [1780558748.244069591] [recovery_server]: Configuring wait
[rtabmap-3] [INFO] [1780558748.324946924] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558748.325096015] [rtabmap]: rtabmap (44): Rate=0.33s, Limit=0.000s, Conversion=0.1372s, RTAB-Map=0.0025s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558748.336236097] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558748.337965858] [rtabmap]: rtabmap (45): Rate=0.33s, Limit=0.000s, Conversion=0.0034s, RTAB-Map=0.0073s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[lifecycle_manager-9] [INFO] [1780558748.399421289] [lifecycle_manager_navigation]: [34m[1mConfiguring bt_navigator[0m[0m
[bt_navigator-8] [INFO] [1780558748.412715140] [bt_navigator]: Configuring
[rtabmap-3] [INFO] [1780558748.438736782] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558748.438877137] [rtabmap]: rtabmap (46): Rate=0.33s, Limit=0.000s, Conversion=0.0735s, RTAB-Map=0.0043s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558748.557743635] [rtabmap]: rtabmap (47): Rate=0.33s, Limit=0.000s, Conversion=0.0775s, RTAB-Map=0.0377s, Maps update=0.0008s pub=0.0002s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558748.637684151] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558748.642662292] [rtabmap]: rtabmap (48): Rate=0.33s, Limit=0.000s, Conversion=0.0686s, RTAB-Map=0.0087s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[bt_navigator-8] [INFO] [1780558748.830597837] [bt_navigator_rclcpp_node]: Waiting for "compute_path_to_pose" action server
[bt_navigator-8] [INFO] [1780558748.834875742] [bt_navigator_rclcpp_node]: "ComputePathToPose" BtActionNode initialized
[rtabmap-3] [INFO] [1780558748.847542861] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558748.847693295] [rtabmap]: rtabmap (49): Rate=0.33s, Limit=0.000s, Conversion=0.1546s, RTAB-Map=0.0025s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[bt_navigator-8] [INFO] [1780558748.907723484] [bt_navigator_rclcpp_node]: Waiting for "follow_path" action server
[bt_navigator-8] [INFO] [1780558748.910463951] [bt_navigator_rclcpp_node]: "FollowPath" BtActionNode initialized
[lifecycle_manager-9] [INFO] [1780558748.925583244] [lifecycle_manager_navigation]: [34m[1mActivating controller_server[0m[0m
[controller_server-5] [INFO] [1780558748.930202659] [controller_server]: Activating
[controller_server-5] [INFO] [1780558748.930339878] [local_costmap.local_costmap]: Activating
[controller_server-5] [INFO] [1780558748.930380551] [local_costmap.local_costmap]: Checking transform
[controller_server-5] [INFO] [1780558748.930620299] [local_costmap.local_costmap]: start
[controller_server-5] [INFO] [1780558748.982341115] [controller_server]: Activating controller: FollowPath of type regulated_pure_pursuit_controller::RegulatedPurePursuitController
[lifecycle_manager-9] [INFO] [1780558748.988969336] [lifecycle_manager_navigation]: [34m[1mActivating planner_server[0m[0m
[planner_server-6] [INFO] [1780558748.992872769] [planner_server]: Activating
[planner_server-6] [INFO] [1780558748.993014628] [global_costmap.global_costmap]: Activating
[planner_server-6] [INFO] [1780558748.993054981] [global_costmap.global_costmap]: Checking transform
[planner_server-6] [INFO] [1780558748.993268233] [global_costmap.global_costmap]: start
[planner_server-6] [INFO] [1780558748.998425194] [planner_server]: Activating plugin GridBased of type NavfnPlanner
[lifecycle_manager-9] [INFO] [1780558749.013048766] [lifecycle_manager_navigation]: [34m[1mActivating recovery_server[0m[0m
[recoveries_server-7] [INFO] [1780558749.017197037] [recovery_server]: Activating
[recoveries_server-7] [INFO] [1780558749.017337647] [recovery_server]: Activating spin
[recoveries_server-7] [INFO] [1780558749.017381104] [recovery_server]: Activating backup
[recoveries_server-7] [INFO] [1780558749.017412209] [recovery_server]: Activating wait
[lifecycle_manager-9] [INFO] [1780558749.024424885] [lifecycle_manager_navigation]: [34m[1mActivating bt_navigator[0m[0m
[bt_navigator-8] [INFO] [1780558749.038070743] [bt_navigator]: Activating
[lifecycle_manager-9] [INFO] [1780558749.050042969] [lifecycle_manager_navigation]: [34m[1mManaged nodes are active[0m[0m
[rtabmap-3] [INFO] [1780558749.090477813] [rtabmap]: rtabmap (50): Rate=0.33s, Limit=0.000s, Conversion=0.1068s, RTAB-Map=0.1350s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558749.126725442] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558749.126882245] [rtabmap]: rtabmap (51): Rate=0.33s, Limit=0.000s, Conversion=0.0308s, RTAB-Map=0.0031s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558749.209109847] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558749.209272090] [rtabmap]: rtabmap (52): Rate=0.33s, Limit=0.000s, Conversion=0.0590s, RTAB-Map=0.0089s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558749.375534591] [rtabmap]: rtabmap (53): Rate=0.33s, Limit=0.000s, Conversion=0.0620s, RTAB-Map=0.0952s, Maps update=0.0003s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558749.422548920] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558749.427402707] [rtabmap]: rtabmap (54): Rate=0.33s, Limit=0.000s, Conversion=0.0380s, RTAB-Map=0.0078s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558749.523680174] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558749.523881426] [rtabmap]: rtabmap (55): Rate=0.33s, Limit=0.000s, Conversion=0.0517s, RTAB-Map=0.0022s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558749.642037163] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558749.642178381] [rtabmap]: rtabmap (56): Rate=0.33s, Limit=0.000s, Conversion=0.1026s, RTAB-Map=0.0022s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558749.732378549] [rtabmap]: rtabmap (57): Rate=0.33s, Limit=0.000s, Conversion=0.0319s, RTAB-Map=0.0574s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558749.737728923] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558749.740258250] [rtabmap]: rtabmap (58): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0046s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558749.961113495] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558749.961279258] [rtabmap]: rtabmap (59): Rate=0.33s, Limit=0.000s, Conversion=0.0330s, RTAB-Map=0.0017s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.221436213] [rtabmap]: rtabmap (60): Rate=0.33s, Limit=0.000s, Conversion=0.0410s, RTAB-Map=0.0666s, Maps update=0.0007s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.263443248] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558750.263790871] [rtabmap]: rtabmap (61): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0177s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.337790544] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558750.337962707] [rtabmap]: rtabmap (62): Rate=0.33s, Limit=0.000s, Conversion=0.0207s, RTAB-Map=0.0063s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.401797163] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558750.401979183] [rtabmap]: rtabmap (63): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0173s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.471785592] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558750.471972636] [rtabmap]: rtabmap (64): Rate=0.33s, Limit=0.000s, Conversion=0.0234s, RTAB-Map=0.0051s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.639940839] [rtabmap]: rtabmap (65): Rate=0.33s, Limit=0.000s, Conversion=0.0639s, RTAB-Map=0.0664s, Maps update=0.0002s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.650122344] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558750.650380684] [rtabmap]: rtabmap (66): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0095s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.722996107] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558750.723165646] [rtabmap]: rtabmap (67): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0061s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.825260603] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558750.825434910] [rtabmap]: rtabmap (68): Rate=0.33s, Limit=0.000s, Conversion=0.0511s, RTAB-Map=0.0021s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.901001300] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558750.901286810] [rtabmap]: rtabmap (69): Rate=0.33s, Limit=0.000s, Conversion=0.0310s, RTAB-Map=0.0322s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.974253087] [rtabmap]: rtabmap (70): Rate=0.33s, Limit=0.000s, Conversion=0.0312s, RTAB-Map=0.0410s, Maps update=0.0002s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558750.978121480] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558750.978283627] [rtabmap]: rtabmap (71): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0031s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.053206615] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558751.053386138] [rtabmap]: rtabmap (72): Rate=0.33s, Limit=0.000s, Conversion=0.0112s, RTAB-Map=0.0185s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.101702447] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558751.101864178] [rtabmap]: rtabmap (73): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0023s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.181052495] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558751.181239827] [rtabmap]: rtabmap (74): Rate=0.33s, Limit=0.000s, Conversion=0.0229s, RTAB-Map=0.0044s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.220294743] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558751.220475995] [rtabmap]: rtabmap (75): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.358310448] [rtabmap]: rtabmap (76): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0712s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.368880121] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558751.369027836] [rtabmap]: rtabmap (77): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0097s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.506725263] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558751.506915634] [rtabmap]: rtabmap (78): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0140s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.765460473] [rtabmap]: rtabmap (79): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0891s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.835502569] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558751.835644236] [rtabmap]: rtabmap (80): Rate=0.33s, Limit=0.000s, Conversion=0.0347s, RTAB-Map=0.0345s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.978375198] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558751.978771142] [rtabmap]: rtabmap (81): Rate=0.33s, Limit=0.000s, Conversion=0.0760s, RTAB-Map=0.0298s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558751.992452969] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558751.992603820] [rtabmap]: rtabmap (82): Rate=0.33s, Limit=0.000s, Conversion=0.0108s, RTAB-Map=0.0023s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.024659565] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558752.024826192] [rtabmap]: rtabmap (83): Rate=0.33s, Limit=0.000s, Conversion=0.0207s, RTAB-Map=0.0026s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.289684791] [rtabmap]: rtabmap (84): Rate=0.33s, Limit=0.000s, Conversion=0.0006s, RTAB-Map=0.0251s, Maps update=0.0002s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.338563928] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558752.340583454] [rtabmap]: rtabmap (85): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.402149264] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558752.402333587] [rtabmap]: rtabmap (86): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0056s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[INFO] [frontier_explorer.py-10]: process started with pid [153091]
[rtabmap-3] [INFO] [1780558752.475474593] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558752.475635588] [rtabmap]: rtabmap (87): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.570951798] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558752.571129626] [rtabmap]: rtabmap (88): Rate=0.33s, Limit=0.000s, Conversion=0.0112s, RTAB-Map=0.0342s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.641690966] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558752.641835577] [rtabmap]: rtabmap (89): Rate=0.33s, Limit=0.000s, Conversion=0.0308s, RTAB-Map=0.0058s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.816676562] [rtabmap]: rtabmap (90): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0829s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.831585965] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558752.831746256] [rtabmap]: rtabmap (91): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0141s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.860734263] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558752.860933755] [rtabmap]: rtabmap (92): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0142s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.940269054] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558752.940443649] [rtabmap]: rtabmap (93): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0282s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558752.979587049] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558752.979741228] [rtabmap]: rtabmap (94): Rate=0.33s, Limit=0.000s, Conversion=0.0013s, RTAB-Map=0.0032s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.067394831] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558753.067613843] [rtabmap]: rtabmap (95): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0234s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.271218198] [rtabmap]: rtabmap (96): Rate=0.33s, Limit=0.000s, Conversion=0.0351s, RTAB-Map=0.1284s, Maps update=0.0003s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.274501236] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558753.274637175] [rtabmap]: rtabmap (97): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0024s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.306637080] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558753.306799899] [rtabmap]: rtabmap (98): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0047s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.390322704] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558753.390511092] [rtabmap]: rtabmap (99): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0115s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.491843612] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558753.492082529] [rtabmap]: rtabmap (100): Rate=0.33s, Limit=0.000s, Conversion=0.0130s, RTAB-Map=0.0414s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.572621853] [rtabmap]: rtabmap (101): Rate=0.33s, Limit=0.000s, Conversion=0.0139s, RTAB-Map=0.0551s, Maps update=0.0002s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.593272294] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558753.593448266] [rtabmap]: rtabmap (102): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0109s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.676055182] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558753.676245361] [rtabmap]: rtabmap (103): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0085s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.707686600] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558753.707833579] [rtabmap]: rtabmap (104): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0028s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558753.837325227] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558753.837479406] [rtabmap]: rtabmap (105): Rate=0.33s, Limit=0.000s, Conversion=0.0550s, RTAB-Map=0.0162s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558754.039475763] [rtabmap]: rtabmap (106): Rate=0.33s, Limit=0.000s, Conversion=0.0618s, RTAB-Map=0.0810s, Maps update=0.0005s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558754.046253780] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558754.046397655] [rtabmap]: rtabmap (107): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0027s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558754.217021964] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558754.217175247] [rtabmap]: rtabmap (108): Rate=0.33s, Limit=0.000s, Conversion=0.0616s, RTAB-Map=0.0027s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558754.281804607] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558754.281966978] [rtabmap]: rtabmap (109): Rate=0.33s, Limit=0.000s, Conversion=0.0602s, RTAB-Map=0.0026s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558754.473784363] [rtabmap]: rtabmap (110): Rate=0.33s, Limit=0.000s, Conversion=0.0741s, RTAB-Map=0.1143s, Maps update=0.0017s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558754.480724335] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558754.480886290] [rtabmap]: rtabmap (111): Rate=0.33s, Limit=0.000s, Conversion=0.0008s, RTAB-Map=0.0036s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558754.550404063] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558754.550584131] [rtabmap]: rtabmap (112): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0031s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558754.623962266] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558754.624157277] [rtabmap]: rtabmap (113): Rate=0.33s, Limit=0.000s, Conversion=0.0106s, RTAB-Map=0.0099s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[frontier_explorer.py-10] [INFO] [1780558754.772329990] [frontier_explorer]: FrontierExplorer started. map_topic=/map, replan=3.0s, min_frontier_size=10 cells, info_gain_radius=1.5m
[rtabmap-3] [INFO] [1780558754.789429644] [rtabmap]: rtabmap (114): Rate=0.33s, Limit=0.000s, Conversion=0.0498s, RTAB-Map=0.0710s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558754.810337531] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558754.810506846] [rtabmap]: rtabmap (115): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0022s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558754.889024055] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558754.889208634] [rtabmap]: rtabmap (116): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0204s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558755.025934378] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558755.026117101] [rtabmap]: rtabmap (117): Rate=0.33s, Limit=0.000s, Conversion=0.0225s, RTAB-Map=0.0087s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558755.116357032] [rtabmap]: rtabmap (118): Rate=0.33s, Limit=0.000s, Conversion=0.0108s, RTAB-Map=0.0436s, Maps update=0.0006s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558755.213432550] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558755.213580681] [rtabmap]: rtabmap (119): Rate=0.33s, Limit=0.000s, Conversion=0.0018s, RTAB-Map=0.0205s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558755.266597086] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558755.266785249] [rtabmap]: rtabmap (120): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0042s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558755.350568257] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558755.350732932] [rtabmap]: rtabmap (121): Rate=0.33s, Limit=0.000s, Conversion=0.0109s, RTAB-Map=0.0134s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558755.516572292] [rtabmap]: rtabmap (122): Rate=0.33s, Limit=0.000s, Conversion=0.0467s, RTAB-Map=0.0218s, Maps update=0.0003s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558755.561213144] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558755.561381595] [rtabmap]: rtabmap (123): Rate=0.33s, Limit=0.000s, Conversion=0.0418s, RTAB-Map=0.0022s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558755.623851332] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558755.623997319] [rtabmap]: rtabmap (124): Rate=0.33s, Limit=0.000s, Conversion=0.0340s, RTAB-Map=0.0059s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558755.646560982] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558755.646733241] [rtabmap]: rtabmap (125): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0023s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558755.782085554] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558755.782259030] [rtabmap]: rtabmap (126): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0024s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.050491130] [rtabmap]: rtabmap (127): Rate=0.33s, Limit=0.000s, Conversion=0.0116s, RTAB-Map=0.1061s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.077837637] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.078027561] [rtabmap]: rtabmap (128): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0266s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.113443406] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.113623730] [rtabmap]: rtabmap (129): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0068s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.235478829] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.235637648] [rtabmap]: rtabmap (130): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0028s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.379465775] [rtabmap]: rtabmap (131): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0741s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.393669823] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.393856003] [rtabmap]: rtabmap (132): Rate=0.33s, Limit=0.000s, Conversion=0.0021s, RTAB-Map=0.0115s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.442644552] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.442857484] [rtabmap]: rtabmap (133): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0083s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.560869310] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.561039585] [rtabmap]: rtabmap (134): Rate=0.33s, Limit=0.000s, Conversion=0.0485s, RTAB-Map=0.0143s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.681152891] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.681332639] [rtabmap]: rtabmap (135): Rate=0.33s, Limit=0.000s, Conversion=0.0649s, RTAB-Map=0.0545s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.721919463] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.722313391] [rtabmap]: rtabmap (136): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0296s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.812488108] [rtabmap]: rtabmap (137): Rate=0.33s, Limit=0.000s, Conversion=0.0367s, RTAB-Map=0.0525s, Maps update=0.0002s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.877420582] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.877584105] [rtabmap]: rtabmap (138): Rate=0.33s, Limit=0.000s, Conversion=0.0619s, RTAB-Map=0.0024s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.936613042] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.936772117] [rtabmap]: rtabmap (139): Rate=0.33s, Limit=0.000s, Conversion=0.0546s, RTAB-Map=0.0040s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558756.980059762] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558756.980257973] [rtabmap]: rtabmap (140): Rate=0.33s, Limit=0.000s, Conversion=0.0315s, RTAB-Map=0.0050s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.189325339] [rtabmap]: rtabmap (141): Rate=0.33s, Limit=0.000s, Conversion=0.0109s, RTAB-Map=0.1006s, Maps update=0.0010s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.226382690] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558757.227411541] [rtabmap]: rtabmap (142): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0363s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.288760653] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558757.288929104] [rtabmap]: rtabmap (143): Rate=0.33s, Limit=0.000s, Conversion=0.0308s, RTAB-Map=0.0277s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.352891514] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558757.353065565] [rtabmap]: rtabmap (144): Rate=0.33s, Limit=0.000s, Conversion=0.0528s, RTAB-Map=0.0107s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.388939693] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558757.389104816] [rtabmap]: rtabmap (145): Rate=0.33s, Limit=0.000s, Conversion=0.0239s, RTAB-Map=0.0115s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.416339322] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558757.416511773] [rtabmap]: rtabmap (146): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0035s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.616365236] [rtabmap]: rtabmap (147): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1454s, Maps update=0.0005s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.628277752] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558757.628456187] [rtabmap]: rtabmap (148): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0112s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.683377464] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558757.683552795] [rtabmap]: rtabmap (149): Rate=0.33s, Limit=0.000s, Conversion=0.0207s, RTAB-Map=0.0028s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.753858239] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558757.754027842] [rtabmap]: rtabmap (150): Rate=0.33s, Limit=0.000s, Conversion=0.0208s, RTAB-Map=0.0076s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.924913645] [rtabmap]: rtabmap (151): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0693s, Maps update=0.0005s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558757.962815139] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558757.963005703] [rtabmap]: rtabmap (152): Rate=0.33s, Limit=0.000s, Conversion=0.0286s, RTAB-Map=0.0068s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.036315685] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558758.036528809] [rtabmap]: rtabmap (153): Rate=0.33s, Limit=0.000s, Conversion=0.0088s, RTAB-Map=0.0392s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.069523907] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558758.069714182] [rtabmap]: rtabmap (154): Rate=0.33s, Limit=0.000s, Conversion=0.0106s, RTAB-Map=0.0104s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.451208546] [rtabmap]: rtabmap (155): Rate=0.33s, Limit=0.000s, Conversion=0.0242s, RTAB-Map=0.1154s, Maps update=0.0003s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.535440243] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558758.535587926] [rtabmap]: rtabmap (156): Rate=0.33s, Limit=0.000s, Conversion=0.0229s, RTAB-Map=0.0061s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.573526415] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558758.574498465] [rtabmap]: rtabmap (157): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0033s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.653567215] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558758.653729362] [rtabmap]: rtabmap (158): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0177s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.755188846] [rtabmap]: rtabmap (159): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0519s, Maps update=0.0003s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.771565737] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558758.771711596] [rtabmap]: rtabmap (160): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0033s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.835235343] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558758.835394866] [rtabmap]: rtabmap (161): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0026s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.900105676] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558758.900292944] [rtabmap]: rtabmap (162): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0038s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558758.989786982] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558758.993378955] [rtabmap]: rtabmap (163): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0162s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.162310044] [rtabmap]: rtabmap (164): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0699s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.183858554] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558759.184007133] [rtabmap]: rtabmap (165): Rate=0.33s, Limit=0.000s, Conversion=0.0112s, RTAB-Map=0.0097s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.240087124] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558759.240234902] [rtabmap]: rtabmap (166): Rate=0.33s, Limit=0.000s, Conversion=0.0111s, RTAB-Map=0.0061s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.291397134] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558759.291564946] [rtabmap]: rtabmap (167): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0024s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.389822899] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558759.389982006] [rtabmap]: rtabmap (168): Rate=0.33s, Limit=0.000s, Conversion=0.0311s, RTAB-Map=0.0075s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.439137032] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558759.439279594] [rtabmap]: rtabmap (169): Rate=0.33s, Limit=0.000s, Conversion=0.0207s, RTAB-Map=0.0025s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.527039426] [rtabmap]: rtabmap (170): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0456s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.579915771] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558759.580089822] [rtabmap]: rtabmap (171): Rate=0.33s, Limit=0.000s, Conversion=0.0210s, RTAB-Map=0.0100s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.698659655] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558759.698805930] [rtabmap]: rtabmap (172): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0160s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.758987375] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558759.759155730] [rtabmap]: rtabmap (173): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0044s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558759.812375122] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558759.812545493] [rtabmap]: rtabmap (174): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0053s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.024642277] [rtabmap]: rtabmap (175): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1483s, Maps update=0.0005s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.030891773] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.031037760] [rtabmap]: rtabmap (176): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0055s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.086333513] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.087332573] [rtabmap]: rtabmap (177): Rate=0.33s, Limit=0.000s, Conversion=0.0107s, RTAB-Map=0.0082s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.148409973] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.148572728] [rtabmap]: rtabmap (178): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0148s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.209254889] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.209405356] [rtabmap]: rtabmap (179): Rate=0.33s, Limit=0.000s, Conversion=0.0108s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.404257526] [rtabmap]: rtabmap (180): Rate=0.33s, Limit=0.000s, Conversion=0.0362s, RTAB-Map=0.0371s, Maps update=0.0002s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.412445332] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.412598455] [rtabmap]: rtabmap (181): Rate=0.33s, Limit=0.000s, Conversion=0.0029s, RTAB-Map=0.0048s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.470776408] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.470941147] [rtabmap]: rtabmap (182): Rate=0.33s, Limit=0.000s, Conversion=0.0119s, RTAB-Map=0.0030s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.535698459] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.535889726] [rtabmap]: rtabmap (183): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0034s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.601796436] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.601977015] [rtabmap]: rtabmap (184): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0139s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.778045207] [rtabmap]: rtabmap (185): Rate=0.33s, Limit=0.000s, Conversion=0.0130s, RTAB-Map=0.0444s, Maps update=0.0010s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.794970461] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.795126528] [rtabmap]: rtabmap (186): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0028s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.853628584] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.853798955] [rtabmap]: rtabmap (187): Rate=0.33s, Limit=0.000s, Conversion=0.0022s, RTAB-Map=0.0044s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558760.930667604] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558760.930871832] [rtabmap]: rtabmap (188): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0076s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.070063444] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558761.071262699] [rtabmap]: rtabmap (189): Rate=0.33s, Limit=0.000s, Conversion=0.0128s, RTAB-Map=0.0100s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.151783389] [rtabmap]: rtabmap (190): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0408s, Maps update=0.0015s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.184887099] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558761.185041566] [rtabmap]: rtabmap (191): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0110s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.338100232] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558761.338362093] [rtabmap]: rtabmap (192): Rate=0.33s, Limit=0.000s, Conversion=0.0107s, RTAB-Map=0.0146s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.387197084] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558761.389514632] [rtabmap]: rtabmap (193): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0147s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.457751629] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558761.457904688] [rtabmap]: rtabmap (194): Rate=0.33s, Limit=0.000s, Conversion=0.0148s, RTAB-Map=0.0090s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.621002716] [rtabmap]: rtabmap (195): Rate=0.33s, Limit=0.000s, Conversion=0.0114s, RTAB-Map=0.1046s, Maps update=0.0005s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.623657583] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558761.625031530] [rtabmap]: rtabmap (196): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0019s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.725864164] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558761.726018759] [rtabmap]: rtabmap (197): Rate=0.33s, Limit=0.000s, Conversion=0.0839s, RTAB-Map=0.0059s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.761798969] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558761.761962940] [rtabmap]: rtabmap (198): Rate=0.33s, Limit=0.000s, Conversion=0.0107s, RTAB-Map=0.0088s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.781527702] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558761.781754554] [rtabmap]: rtabmap (199): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0052s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.841343736] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558761.841519772] [rtabmap]: rtabmap (200): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0072s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558761.970027116] [rtabmap]: rtabmap (201): Rate=0.33s, Limit=0.000s, Conversion=0.0106s, RTAB-Map=0.0643s, Maps update=0.0003s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558762.047093724] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558762.047293600] [rtabmap]: rtabmap (202): Rate=0.33s, Limit=0.000s, Conversion=0.0207s, RTAB-Map=0.0065s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558762.196208767] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558762.196390339] [rtabmap]: rtabmap (203): Rate=0.33s, Limit=0.000s, Conversion=0.0154s, RTAB-Map=0.0175s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558762.660724051] [rtabmap]: rtabmap (204): Rate=0.33s, Limit=0.000s, Conversion=0.0207s, RTAB-Map=0.0363s, Maps update=0.0004s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558762.711181347] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558762.711357510] [rtabmap]: rtabmap (205): Rate=0.33s, Limit=0.000s, Conversion=0.0311s, RTAB-Map=0.0078s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558762.745877825] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558762.746065157] [rtabmap]: rtabmap (206): Rate=0.33s, Limit=0.000s, Conversion=0.0006s, RTAB-Map=0.0039s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558762.898030367] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558762.900840949] [rtabmap]: rtabmap (207): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0094s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558762.960476470] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558762.960670682] [rtabmap]: rtabmap (208): Rate=0.33s, Limit=0.000s, Conversion=0.0161s, RTAB-Map=0.0026s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558763.118068288] [rtabmap]: rtabmap (209): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0463s, Maps update=0.0005s pub=0.0001s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558763.204581996] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558763.204771823] [rtabmap]: rtabmap (210): Rate=0.33s, Limit=0.000s, Conversion=0.0717s, RTAB-Map=0.0021s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558763.245872811] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558763.246110672] [rtabmap]: rtabmap (211): Rate=0.33s, Limit=0.000s, Conversion=0.0348s, RTAB-Map=0.0057s, Maps update=0.0000s pub=0.0000s (local map=1, WM=1)
[rtabmap-3] [INFO] [1780558763.339178010] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558763.339330141] [rtabmap]: rtabmap (212): Rate=0.33s, Limit=0.000s, Conversion=0.0757s, RTAB-Map=0.0087s, Maps update=0.0000s pub=0.0000s (local map=2, WM=2)
[rtabmap-3] [INFO] [1780558763.393610072] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558763.393801340] [rtabmap]: rtabmap (213): Rate=0.33s, Limit=0.000s, Conversion=0.0383s, RTAB-Map=0.0155s, Maps update=0.0000s pub=0.0000s (local map=1, WM=2)
[rtabmap-3] [INFO] [1780558763.490724593] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558763.490928469] [rtabmap]: rtabmap (214): Rate=0.33s, Limit=0.000s, Conversion=0.0788s, RTAB-Map=0.0176s, Maps update=0.0000s pub=0.0000s (local map=1, WM=2)
[rtabmap-3] [INFO] [1780558763.809104294] [rtabmap]: rtabmap (215): Rate=0.33s, Limit=0.000s, Conversion=0.0829s, RTAB-Map=0.2315s, Maps update=0.0024s pub=0.0009s (local map=2, WM=3)
[planner_server-6] [INFO] [1780558763.809475918] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 113 X 102 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558763.869010478] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558763.869214642] [rtabmap]: rtabmap (216): Rate=0.33s, Limit=0.000s, Conversion=0.0513s, RTAB-Map=0.0080s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558764.046267449] [rtabmap]: rtabmap (217): Rate=0.33s, Limit=0.000s, Conversion=0.0643s, RTAB-Map=0.0691s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558764.135370777] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558764.135601149] [rtabmap]: rtabmap (218): Rate=0.33s, Limit=0.000s, Conversion=0.0534s, RTAB-Map=0.0350s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558764.197803444] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558764.197975639] [rtabmap]: rtabmap (219): Rate=0.33s, Limit=0.000s, Conversion=0.0552s, RTAB-Map=0.0065s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558764.282326300] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558764.285848384] [rtabmap]: rtabmap (220): Rate=0.33s, Limit=0.000s, Conversion=0.0338s, RTAB-Map=0.0501s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558764.401306528] [rtabmap]: rtabmap (221): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.1021s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558764.418897813] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558764.419073080] [rtabmap]: rtabmap (222): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0145s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558764.867268463] [rtabmap]: rtabmap (223): Rate=0.33s, Limit=0.000s, Conversion=0.0529s, RTAB-Map=0.0517s, Maps update=0.0002s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558764.876858473] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558764.877064045] [rtabmap]: rtabmap (224): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0088s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558764.909174940] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558764.909363295] [rtabmap]: rtabmap (225): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0120s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558764.964130502] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558764.964324585] [rtabmap]: rtabmap (226): Rate=0.33s, Limit=0.000s, Conversion=0.0108s, RTAB-Map=0.0063s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.035699923] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558765.035906039] [rtabmap]: rtabmap (227): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0192s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.106083211] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558765.106229485] [rtabmap]: rtabmap (228): Rate=0.33s, Limit=0.000s, Conversion=0.0206s, RTAB-Map=0.0019s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.260779977] [rtabmap]: rtabmap (229): Rate=0.33s, Limit=0.000s, Conversion=0.0613s, RTAB-Map=0.0579s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.263103542] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558765.263221912] [rtabmap]: rtabmap (230): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0016s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.273732164] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558765.273878503] [rtabmap]: rtabmap (231): Rate=0.33s, Limit=0.000s, Conversion=0.0006s, RTAB-Map=0.0022s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.435020002] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558765.435188998] [rtabmap]: rtabmap (232): Rate=0.33s, Limit=0.000s, Conversion=0.0210s, RTAB-Map=0.0123s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.485968416] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558765.486108675] [rtabmap]: rtabmap (233): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0026s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.605415281] [rtabmap]: rtabmap (234): Rate=0.33s, Limit=0.000s, Conversion=0.0107s, RTAB-Map=0.0465s, Maps update=0.0012s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.612846050] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558765.612994629] [rtabmap]: rtabmap (235): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0066s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.741302114] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558765.741450533] [rtabmap]: rtabmap (236): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0024s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.800702436] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558765.800839623] [rtabmap]: rtabmap (237): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0029s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.891066048] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558765.891250692] [rtabmap]: rtabmap (238): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0186s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.976066580] [rtabmap]: rtabmap (239): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0478s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558765.999756384] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558766.000390444] [rtabmap]: rtabmap (240): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0067s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.060254041] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558766.060410941] [rtabmap]: rtabmap (241): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.169830254] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558766.169992273] [rtabmap]: rtabmap (242): Rate=0.33s, Limit=0.000s, Conversion=0.0410s, RTAB-Map=0.0045s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.225784111] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558766.225922290] [rtabmap]: rtabmap (243): Rate=0.33s, Limit=0.000s, Conversion=0.0309s, RTAB-Map=0.0113s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.261848397] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558766.262039249] [rtabmap]: rtabmap (244): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0142s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.357178989] [rtabmap]: rtabmap (245): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0346s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.388894038] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558766.389085882] [rtabmap]: rtabmap (246): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0102s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.454644406] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558766.454892475] [rtabmap]: rtabmap (247): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0093s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.531997368] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558766.532164763] [rtabmap]: rtabmap (248): Rate=0.33s, Limit=0.000s, Conversion=0.0208s, RTAB-Map=0.0019s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.610715572] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558766.610888279] [rtabmap]: rtabmap (249): Rate=0.33s, Limit=0.000s, Conversion=0.0212s, RTAB-Map=0.0084s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.913857376] [rtabmap]: rtabmap (250): Rate=0.33s, Limit=0.000s, Conversion=0.0108s, RTAB-Map=0.0590s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558766.917704523] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558766.917864494] [rtabmap]: rtabmap (251): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0030s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.010413976] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.011029156] [rtabmap]: rtabmap (252): Rate=0.33s, Limit=0.000s, Conversion=0.0215s, RTAB-Map=0.0234s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.052501613] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.052685584] [rtabmap]: rtabmap (253): Rate=0.33s, Limit=0.000s, Conversion=0.0115s, RTAB-Map=0.0105s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.097529755] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.097723230] [rtabmap]: rtabmap (254): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0032s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.165230371] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.165394182] [rtabmap]: rtabmap (255): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0075s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.344415656] [rtabmap]: rtabmap (256): Rate=0.33s, Limit=0.000s, Conversion=0.0022s, RTAB-Map=0.0423s, Maps update=0.0003s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.442640483] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.442797990] [rtabmap]: rtabmap (257): Rate=0.33s, Limit=0.000s, Conversion=0.0214s, RTAB-Map=0.0022s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.572761740] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.572906479] [rtabmap]: rtabmap (258): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0236s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.638658929] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.638960567] [rtabmap]: rtabmap (259): Rate=0.33s, Limit=0.000s, Conversion=0.0132s, RTAB-Map=0.0125s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.749285118] [rtabmap]: rtabmap (260): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0676s, Maps update=0.0029s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.758332399] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.758508626] [rtabmap]: rtabmap (261): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0023s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.818898091] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.819119024] [rtabmap]: rtabmap (262): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0101s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.938482951] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.939380729] [rtabmap]: rtabmap (263): Rate=0.33s, Limit=0.000s, Conversion=0.0470s, RTAB-Map=0.0121s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558767.994808049] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558767.994971124] [rtabmap]: rtabmap (264): Rate=0.33s, Limit=0.000s, Conversion=0.0515s, RTAB-Map=0.0032s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.172672227] [rtabmap]: rtabmap (265): Rate=0.33s, Limit=0.000s, Conversion=0.0008s, RTAB-Map=0.1027s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.176042053] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558768.176184583] [rtabmap]: rtabmap (266): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.200051385] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558768.200236541] [rtabmap]: rtabmap (267): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0016s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.276032293] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558768.278358419] [rtabmap]: rtabmap (268): Rate=0.33s, Limit=0.000s, Conversion=0.0111s, RTAB-Map=0.0024s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.402539436] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558768.402705647] [rtabmap]: rtabmap (269): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0099s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.500033372] [rtabmap]: rtabmap (270): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0391s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.562822022] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558768.562996522] [rtabmap]: rtabmap (271): Rate=0.33s, Limit=0.000s, Conversion=0.0131s, RTAB-Map=0.0241s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.703589604] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558768.703892394] [rtabmap]: rtabmap (272): Rate=0.33s, Limit=0.000s, Conversion=0.0106s, RTAB-Map=0.0375s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.805009984] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558768.805190340] [rtabmap]: rtabmap (273): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0092s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.919118869] [rtabmap]: rtabmap (274): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0581s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558768.993779047] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558768.993930666] [rtabmap]: rtabmap (275): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0028s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558769.069206027] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558769.069393390] [rtabmap]: rtabmap (276): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0144s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558769.124348385] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558769.124541477] [rtabmap]: rtabmap (277): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0053s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558769.176529694] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558769.176714818] [rtabmap]: rtabmap (278): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558769.412267714] [rtabmap]: rtabmap (279): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.1517s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558769.424018920] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558769.424187019] [rtabmap]: rtabmap (280): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0110s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558769.460332430] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558769.460532722] [rtabmap]: rtabmap (281): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0151s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558769.581907513] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558769.582082044] [rtabmap]: rtabmap (282): Rate=0.33s, Limit=0.000s, Conversion=0.0007s, RTAB-Map=0.0130s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558769.756784054] [rtabmap]: rtabmap (283): Rate=0.33s, Limit=0.000s, Conversion=0.0111s, RTAB-Map=0.1095s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558769.766405362] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558769.766582677] [rtabmap]: rtabmap (284): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0087s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558769.930761249] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558769.930936581] [rtabmap]: rtabmap (285): Rate=0.33s, Limit=0.000s, Conversion=0.0969s, RTAB-Map=0.0023s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558770.246479946] [rtabmap]: rtabmap (286): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0838s, Maps update=0.0005s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558770.259884752] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558770.260103701] [rtabmap]: rtabmap (287): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0125s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558770.293343232] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558770.293542756] [rtabmap]: rtabmap (288): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0077s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558770.400279632] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558770.400452563] [rtabmap]: rtabmap (289): Rate=0.33s, Limit=0.000s, Conversion=0.0357s, RTAB-Map=0.0051s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558770.435655365] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558770.435884041] [rtabmap]: rtabmap (290): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0171s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558770.500612894] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558770.500791041] [rtabmap]: rtabmap (291): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0027s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[frontier_explorer.py-10] [INFO] [1780558770.638985365] [frontier_explorer]: Selected frontier: (-3.07,-1.76) score=9.07 gain=248 size=612 d=0.24m turn=80deg clear=0.30m from 4 candidates
[bt_navigator-8] [INFO] [1780558770.661497454] [bt_navigator]: Begin navigating from current location to (-3.07, -1.76)
[controller_server-5] [INFO] [1780558770.709811073] [controller_server]: Received a goal, begin computing control effort.
[rtabmap-3] [INFO] [1780558770.740468506] [rtabmap]: rtabmap (292): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1939s, Maps update=0.0004s pub=0.0001s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558770.813796759] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558770.813979962] [rtabmap]: rtabmap (293): Rate=0.33s, Limit=0.000s, Conversion=0.0572s, RTAB-Map=0.0112s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558770.882941218] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558770.883073092] [rtabmap]: rtabmap (294): Rate=0.33s, Limit=0.000s, Conversion=0.0586s, RTAB-Map=0.0097s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[rtabmap-3] [INFO] [1780558770.958278342] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558770.958469642] [rtabmap]: rtabmap (295): Rate=0.33s, Limit=0.000s, Conversion=0.0587s, RTAB-Map=0.0161s, Maps update=0.0000s pub=0.0000s (local map=2, WM=3)
[planner_server-6] [INFO] [1780558771.260694485] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 113 X 106 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558771.265110475] [rtabmap]: rtabmap (296): Rate=0.33s, Limit=0.000s, Conversion=0.0456s, RTAB-Map=0.2048s, Maps update=0.0044s pub=0.0060s (local map=3, WM=4)
[rtabmap-3] [INFO] [1780558771.330135560] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558771.330309035] [rtabmap]: rtabmap (297): Rate=0.33s, Limit=0.000s, Conversion=0.0528s, RTAB-Map=0.0115s, Maps update=0.0000s pub=0.0000s (local map=4, WM=5)
[rtabmap-3] [INFO] [1780558771.654504583] [rtabmap]: rtabmap (298): Rate=0.33s, Limit=0.000s, Conversion=0.0932s, RTAB-Map=0.1718s, Maps update=0.0023s pub=0.0010s (local map=4, WM=6)
[planner_server-6] [INFO] [1780558771.654694730] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 113 X 110 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558771.738255506] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558771.738423254] [rtabmap]: rtabmap (299): Rate=0.33s, Limit=0.000s, Conversion=0.0715s, RTAB-Map=0.0118s, Maps update=0.0000s pub=0.0000s (local map=5, WM=7)
[rtabmap-3] [INFO] [1780558771.798578195] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558771.798743126] [rtabmap]: rtabmap (300): Rate=0.33s, Limit=0.000s, Conversion=0.0512s, RTAB-Map=0.0085s, Maps update=0.0000s pub=0.0000s (local map=4, WM=7)
[controller_server-5] [INFO] [1780558771.912092231] [controller_server]: Passing new path to controller.
[planner_server-6] [INFO] [1780558772.090709436] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 118 X 110 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558772.093170284] [rtabmap]: rtabmap (301): Rate=0.33s, Limit=0.000s, Conversion=0.0435s, RTAB-Map=0.2453s, Maps update=0.0009s pub=0.0040s (local map=5, WM=8)
[rtabmap-3] [INFO] [1780558772.119507986] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558772.121255604] [rtabmap]: rtabmap (302): Rate=0.33s, Limit=0.000s, Conversion=0.0015s, RTAB-Map=0.0226s, Maps update=0.0000s pub=0.0000s (local map=6, WM=9)
[rtabmap-3] [INFO] [1780558772.127834741] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558772.129933663] [rtabmap]: rtabmap (303): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0021s, Maps update=0.0000s pub=0.0000s (local map=6, WM=10)
[rtabmap-3] [INFO] [1780558772.395203230] [rtabmap]: rtabmap (304): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.2062s, Maps update=0.0022s pub=0.0021s (local map=6, WM=11)
[rtabmap-3] [INFO] [1780558772.437855909] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558772.438015144] [rtabmap]: rtabmap (305): Rate=0.33s, Limit=0.000s, Conversion=0.0375s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=7, WM=12)
[rtabmap-3] [INFO] [1780558772.477331693] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558772.477499440] [rtabmap]: rtabmap (306): Rate=0.33s, Limit=0.000s, Conversion=0.0308s, RTAB-Map=0.0081s, Maps update=0.0000s pub=0.0000s (local map=6, WM=12)
[rtabmap-3] [INFO] [1780558772.585802242] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558772.585959973] [rtabmap]: rtabmap (307): Rate=0.33s, Limit=0.000s, Conversion=0.0616s, RTAB-Map=0.0187s, Maps update=0.0000s pub=0.0000s (local map=7, WM=13)
[rtabmap-3] [INFO] [1780558772.843884148] [rtabmap]: rtabmap (308): Rate=0.33s, Limit=0.000s, Conversion=0.0318s, RTAB-Map=0.2238s, Maps update=0.0005s pub=0.0014s (local map=7, WM=14)
[planner_server-6] [INFO] [1780558772.845064555] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 120 X 110 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558772.849619013] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558772.850363379] [rtabmap]: rtabmap (309): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0048s, Maps update=0.0000s pub=0.0000s (local map=8, WM=15)
[controller_server-5] [INFO] [1780558772.912541306] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558772.937457668] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558772.937617095] [rtabmap]: rtabmap (310): Rate=0.33s, Limit=0.000s, Conversion=0.0427s, RTAB-Map=0.0114s, Maps update=0.0000s pub=0.0000s (local map=8, WM=16)
[rtabmap-3] [INFO] [1780558773.288065562] [rtabmap]: rtabmap (311): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1876s, Maps update=0.0186s pub=0.0010s (local map=8, WM=17)
[rtabmap-3] [INFO] [1780558773.292160491] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558773.293623432] [rtabmap]: rtabmap (312): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0033s, Maps update=0.0000s pub=0.0000s (local map=9, WM=18)
[rtabmap-3] [INFO] [1780558773.450921186] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558773.451080517] [rtabmap]: rtabmap (313): Rate=0.33s, Limit=0.000s, Conversion=0.0329s, RTAB-Map=0.0105s, Maps update=0.0000s pub=0.0000s (local map=8, WM=18)
[rtabmap-3] [INFO] [1780558773.884599070] [rtabmap]: rtabmap (314): Rate=0.33s, Limit=0.000s, Conversion=0.0639s, RTAB-Map=0.1865s, Maps update=0.0290s pub=0.0020s (local map=9, WM=19)
[rtabmap-3] [INFO] [1780558773.897801602] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558773.897980038] [rtabmap]: rtabmap (315): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0124s, Maps update=0.0000s pub=0.0000s (local map=10, WM=20)
[controller_server-5] [INFO] [1780558773.912085852] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558774.254808097] [rtabmap]: rtabmap (316): Rate=0.33s, Limit=0.000s, Conversion=0.0111s, RTAB-Map=0.1868s, Maps update=0.0008s pub=0.0012s (local map=10, WM=21)
[planner_server-6] [INFO] [1780558774.255350924] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 121 X 110 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558774.272014645] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558774.272170680] [rtabmap]: rtabmap (317): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0163s, Maps update=0.0000s pub=0.0000s (local map=10, WM=21)
[rtabmap-3] [INFO] [1780558774.287863694] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558774.288009904] [rtabmap]: rtabmap (318): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0122s, Maps update=0.0000s pub=0.0000s (local map=10, WM=21)
[rtabmap-3] [INFO] [1780558774.408445304] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558774.408627612] [rtabmap]: rtabmap (319): Rate=0.33s, Limit=0.000s, Conversion=0.0214s, RTAB-Map=0.0058s, Maps update=0.0000s pub=0.0000s (local map=11, WM=22)
[rtabmap-3] [INFO] [1780558774.658720129] [rtabmap]: rtabmap (320): Rate=0.33s, Limit=0.000s, Conversion=0.0534s, RTAB-Map=0.1512s, Maps update=0.0018s pub=0.0030s (local map=11, WM=23)
[planner_server-6] [INFO] [1780558774.659465008] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 121 X 111 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558774.707755784] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558774.707902859] [rtabmap]: rtabmap (321): Rate=0.33s, Limit=0.000s, Conversion=0.0018s, RTAB-Map=0.0460s, Maps update=0.0000s pub=0.0000s (local map=11, WM=23)
[rtabmap-3] [INFO] [1780558774.838247159] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558774.838411738] [rtabmap]: rtabmap (322): Rate=0.33s, Limit=0.000s, Conversion=0.0539s, RTAB-Map=0.0060s, Maps update=0.0000s pub=0.0000s (local map=12, WM=24)
[controller_server-5] [INFO] [1780558774.912102823] [controller_server]: Passing new path to controller.
[controller_server-5] [INFO] [1780558774.913176573] [controller_server]: Reached the goal!
[bt_navigator-8] [INFO] [1780558774.954571693] [bt_navigator]: Navigation succeeded
[frontier_explorer.py-10] [INFO] [1780558775.023839940] [frontier_explorer]: Reached frontier (-3.068424541503191, -1.758538905531168), looking for next.
[rtabmap-3] [INFO] [1780558775.053197160] [rtabmap]: rtabmap (323): Rate=0.33s, Limit=0.000s, Conversion=0.0343s, RTAB-Map=0.1694s, Maps update=0.0009s pub=0.0017s (local map=12, WM=25)
[planner_server-6] [INFO] [1780558775.057482621] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 122 X 111 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558775.074132230] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558775.074297289] [rtabmap]: rtabmap (324): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0099s, Maps update=0.0000s pub=0.0000s (local map=12, WM=25)
[rtabmap-3] [INFO] [1780558775.114841034] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558775.115012814] [rtabmap]: rtabmap (325): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0069s, Maps update=0.0000s pub=0.0000s (local map=12, WM=25)
[rtabmap-3] [INFO] [1780558775.322453553] [rtabmap]: rtabmap (326): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0836s, Maps update=0.0005s pub=0.0001s (local map=12, WM=25)
[rtabmap-3] [INFO] [1780558775.327158894] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558775.327322833] [rtabmap]: rtabmap (327): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0040s, Maps update=0.0000s pub=0.0000s (local map=12, WM=25)
[rtabmap-3] [INFO] [1780558775.366464982] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558775.366616633] [rtabmap]: rtabmap (328): Rate=0.33s, Limit=0.000s, Conversion=0.0049s, RTAB-Map=0.0032s, Maps update=0.0000s pub=0.0000s (local map=12, WM=25)
[rtabmap-3] [INFO] [1780558775.444500124] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558775.444659551] [rtabmap]: rtabmap (329): Rate=0.33s, Limit=0.000s, Conversion=0.0207s, RTAB-Map=0.0028s, Maps update=0.0000s pub=0.0000s (local map=12, WM=25)
[cmd_vel_bridge.py-4] [INFO] [1780558775.458046376] [cmd_vel_bridge]: cmd_vel timeout, sending stop
[rtabmap-3] [INFO] [1780558775.516238886] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558775.516416745] [rtabmap]: rtabmap (330): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0203s, Maps update=0.0000s pub=0.0000s (local map=12, WM=25)
[rtabmap-3] [INFO] [1780558775.561452035] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558775.561621862] [rtabmap]: rtabmap (331): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0068s, Maps update=0.0000s pub=0.0000s (local map=12, WM=25)
[rtabmap-3] [INFO] [1780558775.687583359] [rtabmap]: rtabmap (332): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0708s, Maps update=0.0005s pub=0.0001s (local map=12, WM=25)
[rtabmap-3] [INFO] [1780558775.700791076] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558775.700945319] [rtabmap]: rtabmap (333): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0022s, Maps update=0.0000s pub=0.0000s (local map=12, WM=25)
[frontier_explorer.py-10] [INFO] [1780558775.742101236] [frontier_explorer]: Selected frontier: (-4.77,0.14) score=7.15 gain=522 size=141 d=2.56m turn=85deg clear=0.42m from 14 candidates
[bt_navigator-8] [INFO] [1780558775.749589032] [bt_navigator]: Begin navigating from current location to (-4.77, 0.14)
[rtabmap-3] [INFO] [1780558775.788545258] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558775.788700013] [rtabmap]: rtabmap (334): Rate=0.33s, Limit=0.000s, Conversion=0.0316s, RTAB-Map=0.0111s, Maps update=0.0000s pub=0.0000s (local map=12, WM=25)
[controller_server-5] [INFO] [1780558775.795634326] [controller_server]: Received a goal, begin computing control effort.
[controller_server-5] [INFO] [1780558775.795855962] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558775.880924715] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558775.881114191] [rtabmap]: rtabmap (335): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0041s, Maps update=0.0000s pub=0.0000s (local map=12, WM=25)
[rtabmap-3] [INFO] [1780558776.135512886] [rtabmap]: rtabmap (336): Rate=0.33s, Limit=0.000s, Conversion=0.0107s, RTAB-Map=0.0526s, Maps update=0.0004s pub=0.0001s (local map=12, WM=25)
[rtabmap-3] [INFO] [1780558776.179075988] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558776.179251512] [rtabmap]: rtabmap (337): Rate=0.33s, Limit=0.000s, Conversion=0.0309s, RTAB-Map=0.0100s, Maps update=0.0000s pub=0.0000s (local map=13, WM=26)
[rtabmap-3] [INFO] [1780558776.284837985] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558776.285318699] [rtabmap]: rtabmap (338): Rate=0.33s, Limit=0.000s, Conversion=0.0721s, RTAB-Map=0.0081s, Maps update=0.0000s pub=0.0000s (local map=12, WM=26)
[rtabmap-3] [INFO] [1780558776.341883050] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558776.342078478] [rtabmap]: rtabmap (339): Rate=0.33s, Limit=0.000s, Conversion=0.0263s, RTAB-Map=0.0297s, Maps update=0.0000s pub=0.0000s (local map=13, WM=27)
[rtabmap-3] [INFO] [1780558776.411553773] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558776.411923572] [rtabmap]: rtabmap (340): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0055s, Maps update=0.0000s pub=0.0000s (local map=13, WM=28)
[rtabmap-3] [INFO] [1780558776.622194774] [rtabmap]: rtabmap (341): Rate=0.33s, Limit=0.000s, Conversion=0.0256s, RTAB-Map=0.1332s, Maps update=0.0009s pub=0.0015s (local map=13, WM=29)
[rtabmap-3] [INFO] [1780558776.630861377] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558776.631013476] [rtabmap]: rtabmap (342): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0078s, Maps update=0.0000s pub=0.0000s (local map=14, WM=30)
[rtabmap-3] [INFO] [1780558776.677133525] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558776.677346681] [rtabmap]: rtabmap (343): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0158s, Maps update=0.0000s pub=0.0000s (local map=13, WM=30)
[rtabmap-3] [INFO] [1780558776.781812749] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558776.782000081] [rtabmap]: rtabmap (344): Rate=0.33s, Limit=0.000s, Conversion=0.0399s, RTAB-Map=0.0151s, Maps update=0.0000s pub=0.0000s (local map=14, WM=31)
[rtabmap-3] [INFO] [1780558776.808924358] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558776.809106825] [rtabmap]: rtabmap (345): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0042s, Maps update=0.0000s pub=0.0000s (local map=13, WM=31)
[controller_server-5] [INFO] [1780558776.996036925] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558777.262478110] [rtabmap]: rtabmap (346): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.3394s, Maps update=0.0009s pub=0.0051s (local map=14, WM=32)
[rtabmap-3] [INFO] [1780558777.307607549] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558777.307810433] [rtabmap]: rtabmap (347): Rate=0.33s, Limit=0.000s, Conversion=0.0110s, RTAB-Map=0.0334s, Maps update=0.0000s pub=0.0000s (local map=15, WM=33)
[rtabmap-3] [INFO] [1780558777.638652767] [rtabmap]: rtabmap (348): Rate=0.33s, Limit=0.000s, Conversion=0.0036s, RTAB-Map=0.3234s, Maps update=0.0010s pub=0.0021s (local map=15, WM=34)
[planner_server-6] [INFO] [1780558777.642657807] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 122 X 126 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558777.717408313] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558777.717627229] [rtabmap]: rtabmap (349): Rate=0.33s, Limit=0.000s, Conversion=0.0632s, RTAB-Map=0.0122s, Maps update=0.0000s pub=0.0000s (local map=16, WM=35)
[controller_server-5] [INFO] [1780558777.996187119] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558778.124390400] [rtabmap]: rtabmap (350): Rate=0.33s, Limit=0.000s, Conversion=0.0520s, RTAB-Map=0.3485s, Maps update=0.0033s pub=0.0023s (local map=16, WM=36)
[rtabmap-3] [INFO] [1780558778.138561562] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558778.139094692] [rtabmap]: rtabmap (351): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0133s, Maps update=0.0000s pub=0.0000s (local map=17, WM=37)
[rtabmap-3] [INFO] [1780558778.518200172] [rtabmap]: rtabmap (352): Rate=0.33s, Limit=0.000s, Conversion=0.0516s, RTAB-Map=0.1735s, Maps update=0.0009s pub=0.0023s (local map=17, WM=38)
[rtabmap-3] [INFO] [1780558778.525236183] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558778.525414363] [rtabmap]: rtabmap (353): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0057s, Maps update=0.0000s pub=0.0000s (local map=18, WM=39)
[rtabmap-3] [INFO] [1780558778.560240847] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558778.560405362] [rtabmap]: rtabmap (354): Rate=0.33s, Limit=0.000s, Conversion=0.0126s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=17, WM=39)
[rtabmap-3] [INFO] [1780558778.899871206] [rtabmap]: rtabmap (355): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.2162s, Maps update=0.0010s pub=0.0047s (local map=18, WM=40)
[planner_server-6] [INFO] [1780558778.901981232] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 122 X 127 at 0.100000 m/pix
[controller_server-5] [INFO] [1780558778.997118769] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558779.297999185] [rtabmap]: rtabmap (356): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.2137s, Maps update=0.0098s pub=0.0104s (local map=19, WM=41)
[rtabmap-3] [INFO] [1780558779.312590515] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558779.312778615] [rtabmap]: rtabmap (357): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0137s, Maps update=0.0000s pub=0.0000s (local map=20, WM=42)
[rtabmap-3] [INFO] [1780558779.354778682] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558779.354946013] [rtabmap]: rtabmap (358): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0144s, Maps update=0.0000s pub=0.0000s (local map=19, WM=42)
[rtabmap-3] [INFO] [1780558779.406590240] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558779.406764804] [rtabmap]: rtabmap (359): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0100s, Maps update=0.0000s pub=0.0000s (local map=20, WM=43)
[rtabmap-3] [INFO] [1780558779.653955832] [rtabmap]: rtabmap (360): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.1820s, Maps update=0.0010s pub=0.0052s (local map=20, WM=44)
[planner_server-6] [INFO] [1780558779.654023417] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 122 X 128 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558779.664752975] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558779.664902514] [rtabmap]: rtabmap (361): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0100s, Maps update=0.0000s pub=0.0000s (local map=20, WM=44)
[rtabmap-3] [INFO] [1780558779.800758464] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558779.800900866] [rtabmap]: rtabmap (362): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0179s, Maps update=0.0000s pub=0.0000s (local map=21, WM=45)
[controller_server-5] [INFO] [1780558779.995898889] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558780.143670632] [rtabmap]: rtabmap (363): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.2899s, Maps update=0.0008s pub=0.0015s (local map=21, WM=46)
[rtabmap-3] [INFO] [1780558780.167316575] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558780.167479106] [rtabmap]: rtabmap (364): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0218s, Maps update=0.0000s pub=0.0000s (local map=22, WM=47)
[rtabmap-3] [INFO] [1780558780.188276993] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558780.188426404] [rtabmap]: rtabmap (365): Rate=0.33s, Limit=0.000s, Conversion=0.0106s, RTAB-Map=0.0075s, Maps update=0.0000s pub=0.0000s (local map=21, WM=47)
[rtabmap-3] [INFO] [1780558780.510698615] [rtabmap]: rtabmap (366): Rate=0.33s, Limit=0.000s, Conversion=0.0332s, RTAB-Map=0.2216s, Maps update=0.0093s pub=0.0041s (local map=22, WM=48)
[rtabmap-3] [INFO] [1780558780.546368318] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558780.546542081] [rtabmap]: rtabmap (367): Rate=0.33s, Limit=0.000s, Conversion=0.0308s, RTAB-Map=0.0028s, Maps update=0.0000s pub=0.0000s (local map=22, WM=48)
[rtabmap-3] [INFO] [1780558780.597284884] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558780.597430839] [rtabmap]: rtabmap (368): Rate=0.33s, Limit=0.000s, Conversion=0.0319s, RTAB-Map=0.0018s, Maps update=0.0000s pub=0.0000s (local map=22, WM=48)
[rtabmap-3] [INFO] [1780558780.669340335] [rtabmap]: rtabmap (369): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0419s, Maps update=0.0004s pub=0.0001s (local map=22, WM=48)
[rtabmap-3] [INFO] [1780558780.739456164] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558780.739640456] [rtabmap]: rtabmap (370): Rate=0.33s, Limit=0.000s, Conversion=0.0415s, RTAB-Map=0.0087s, Maps update=0.0000s pub=0.0000s (local map=23, WM=49)
[rtabmap-3] [INFO] [1780558780.903068703] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558780.903211714] [rtabmap]: rtabmap (371): Rate=0.33s, Limit=0.000s, Conversion=0.0704s, RTAB-Map=0.0032s, Maps update=0.0000s pub=0.0000s (local map=22, WM=49)
[rtabmap-3] [INFO] [1780558780.994662976] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558780.994799875] [rtabmap]: rtabmap (372): Rate=0.33s, Limit=0.000s, Conversion=0.0331s, RTAB-Map=0.0072s, Maps update=0.0000s pub=0.0000s (local map=23, WM=50)
[controller_server-5] [INFO] [1780558780.996985454] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558781.185593377] [rtabmap]: rtabmap (373): Rate=0.33s, Limit=0.000s, Conversion=0.0007s, RTAB-Map=0.1495s, Maps update=0.0029s pub=0.0062s (local map=23, WM=51)
[rtabmap-3] [INFO] [1780558781.215005964] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558781.219825964] [rtabmap]: rtabmap (374): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0286s, Maps update=0.0000s pub=0.0000s (local map=23, WM=51)
[rtabmap-3] [INFO] [1780558781.277257958] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558781.277409801] [rtabmap]: rtabmap (375): Rate=0.33s, Limit=0.000s, Conversion=0.0446s, RTAB-Map=0.0109s, Maps update=0.0000s pub=0.0000s (local map=23, WM=51)
[rtabmap-3] [INFO] [1780558781.303027048] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558781.303164267] [rtabmap]: rtabmap (376): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0182s, Maps update=0.0000s pub=0.0000s (local map=23, WM=51)
[rtabmap-3] [INFO] [1780558781.367310059] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558781.367581936] [rtabmap]: rtabmap (377): Rate=0.33s, Limit=0.000s, Conversion=0.0010s, RTAB-Map=0.0193s, Maps update=0.0000s pub=0.0000s (local map=23, WM=51)
[rtabmap-3] [INFO] [1780558781.551259393] [rtabmap]: rtabmap (378): Rate=0.33s, Limit=0.000s, Conversion=0.0113s, RTAB-Map=0.1320s, Maps update=0.0004s pub=0.0001s (local map=23, WM=51)
[rtabmap-3] [INFO] [1780558781.589786241] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558781.589944549] [rtabmap]: rtabmap (379): Rate=0.33s, Limit=0.000s, Conversion=0.0207s, RTAB-Map=0.0173s, Maps update=0.0000s pub=0.0000s (local map=24, WM=52)
[rtabmap-3] [INFO] [1780558781.750034431] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558781.750190082] [rtabmap]: rtabmap (380): Rate=0.33s, Limit=0.000s, Conversion=0.0107s, RTAB-Map=0.0094s, Maps update=0.0000s pub=0.0000s (local map=24, WM=53)
[controller_server-5] [INFO] [1780558782.001709468] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558782.040169853] [rtabmap]: rtabmap (381): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.1676s, Maps update=0.0007s pub=0.0026s (local map=24, WM=54)
[rtabmap-3] [INFO] [1780558782.046857922] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558782.047005701] [rtabmap]: rtabmap (382): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0046s, Maps update=0.0000s pub=0.0000s (local map=24, WM=54)
[rtabmap-3] [INFO] [1780558782.106508650] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558782.106663789] [rtabmap]: rtabmap (383): Rate=0.33s, Limit=0.000s, Conversion=0.0430s, RTAB-Map=0.0076s, Maps update=0.0000s pub=0.0000s (local map=25, WM=55)
[rtabmap-3] [INFO] [1780558782.189531366] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558782.189698249] [rtabmap]: rtabmap (384): Rate=0.33s, Limit=0.000s, Conversion=0.0560s, RTAB-Map=0.0065s, Maps update=0.0000s pub=0.0000s (local map=24, WM=55)
[rtabmap-3] [INFO] [1780558782.217357138] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558782.217508149] [rtabmap]: rtabmap (385): Rate=0.33s, Limit=0.000s, Conversion=0.0117s, RTAB-Map=0.0153s, Maps update=0.0000s pub=0.0000s (local map=24, WM=55)
[rtabmap-3] [INFO] [1780558782.437127354] [rtabmap]: rtabmap (386): Rate=0.33s, Limit=0.000s, Conversion=0.0005s, RTAB-Map=0.1801s, Maps update=0.0035s pub=0.0029s (local map=25, WM=56)
[planner_server-6] [INFO] [1780558782.437289053] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 122 X 129 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558782.445745702] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558782.445909513] [rtabmap]: rtabmap (387): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0078s, Maps update=0.0000s pub=0.0000s (local map=25, WM=56)
[rtabmap-3] [INFO] [1780558782.510007914] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558782.510183501] [rtabmap]: rtabmap (388): Rate=0.33s, Limit=0.000s, Conversion=0.0527s, RTAB-Map=0.0050s, Maps update=0.0000s pub=0.0000s (local map=25, WM=56)
[rtabmap-3] [INFO] [1780558782.596583212] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558782.596756528] [rtabmap]: rtabmap (389): Rate=0.33s, Limit=0.000s, Conversion=0.0544s, RTAB-Map=0.0265s, Maps update=0.0000s pub=0.0000s (local map=25, WM=56)
[rtabmap-3] [INFO] [1780558783.020251391] [rtabmap]: rtabmap (390): Rate=0.33s, Limit=0.000s, Conversion=0.0622s, RTAB-Map=0.2489s, Maps update=0.0009s pub=0.0016s (local map=26, WM=57)
[rtabmap-3] [INFO] [1780558783.028466948] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558783.028642311] [rtabmap]: rtabmap (391): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0074s, Maps update=0.0000s pub=0.0000s (local map=26, WM=57)
[rtabmap-3] [INFO] [1780558783.052821803] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558783.052976142] [rtabmap]: rtabmap (392): Rate=0.33s, Limit=0.000s, Conversion=0.0106s, RTAB-Map=0.0096s, Maps update=0.0000s pub=0.0000s (local map=27, WM=58)
[controller_server-5] [INFO] [1780558783.195967260] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558783.412471466] [rtabmap]: rtabmap (393): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.3063s, Maps update=0.0048s pub=0.0026s (local map=27, WM=59)
[rtabmap-3] [INFO] [1780558783.429075862] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558783.429276026] [rtabmap]: rtabmap (394): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0128s, Maps update=0.0000s pub=0.0000s (local map=27, WM=59)
[rtabmap-3] [INFO] [1780558783.492844531] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558783.493019190] [rtabmap]: rtabmap (395): Rate=0.33s, Limit=0.000s, Conversion=0.0600s, RTAB-Map=0.0030s, Maps update=0.0000s pub=0.0000s (local map=27, WM=59)
[rtabmap-3] [INFO] [1780558783.869336227] [rtabmap]: rtabmap (396): Rate=0.33s, Limit=0.000s, Conversion=0.0488s, RTAB-Map=0.3241s, Maps update=0.0008s pub=0.0019s (local map=28, WM=60)
[rtabmap-3] [INFO] [1780558783.913521559] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558783.913747227] [rtabmap]: rtabmap (397): Rate=0.33s, Limit=0.000s, Conversion=0.0214s, RTAB-Map=0.0215s, Maps update=0.0000s pub=0.0000s (local map=29, WM=61)
[controller_server-5] [INFO] [1780558784.195983955] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558784.440549018] [rtabmap]: rtabmap (398): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.3566s, Maps update=0.0011s pub=0.0020s (local map=29, WM=62)
[rtabmap-3] [INFO] [1780558784.498391297] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558784.498560741] [rtabmap]: rtabmap (399): Rate=0.33s, Limit=0.000s, Conversion=0.0512s, RTAB-Map=0.0060s, Maps update=0.0000s pub=0.0000s (local map=29, WM=62)
[rtabmap-3] [INFO] [1780558784.605452068] [rtabmap]: rtabmap (400): Rate=0.33s, Limit=0.000s, Conversion=0.0630s, RTAB-Map=0.0423s, Maps update=0.0010s pub=0.0001s (local map=29, WM=62)
[rtabmap-3] [INFO] [1780558784.679277452] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558784.679492432] [rtabmap]: rtabmap (401): Rate=0.33s, Limit=0.000s, Conversion=0.0579s, RTAB-Map=0.0128s, Maps update=0.0000s pub=0.0000s (local map=30, WM=63)
[rtabmap-3] [INFO] [1780558784.727163148] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558784.727336847] [rtabmap]: rtabmap (402): Rate=0.33s, Limit=0.000s, Conversion=0.0210s, RTAB-Map=0.0260s, Maps update=0.0000s pub=0.0000s (local map=29, WM=63)
[rtabmap-3] [INFO] [1780558784.737145876] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558784.737322232] [rtabmap]: rtabmap (403): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0091s, Maps update=0.0000s pub=0.0000s (local map=29, WM=63)
[rtabmap-3] [INFO] [1780558784.877778711] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558784.877981211] [rtabmap]: rtabmap (404): Rate=0.33s, Limit=0.000s, Conversion=0.0759s, RTAB-Map=0.0080s, Maps update=0.0000s pub=0.0000s (local map=29, WM=63)
[controller_server-5] [INFO] [1780558785.195976969] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558785.246037942] [rtabmap]: rtabmap (405): Rate=0.33s, Limit=0.000s, Conversion=0.0597s, RTAB-Map=0.2763s, Maps update=0.0284s pub=0.0033s (local map=30, WM=64)
[rtabmap-3] [INFO] [1780558785.293497391] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558785.293734164] [rtabmap]: rtabmap (406): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0463s, Maps update=0.0000s pub=0.0000s (local map=30, WM=64)
[rtabmap-3] [INFO] [1780558785.535442985] [rtabmap]: rtabmap (407): Rate=0.33s, Limit=0.000s, Conversion=0.0208s, RTAB-Map=0.2124s, Maps update=0.0051s pub=0.0025s (local map=31, WM=65)
[rtabmap-3] [INFO] [1780558785.564346061] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558785.564507825] [rtabmap]: rtabmap (408): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0205s, Maps update=0.0000s pub=0.0000s (local map=32, WM=66)
[rtabmap-3] [INFO] [1780558785.609668444] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558785.609868192] [rtabmap]: rtabmap (409): Rate=0.33s, Limit=0.000s, Conversion=0.0112s, RTAB-Map=0.0194s, Maps update=0.0000s pub=0.0000s (local map=31, WM=66)
[planner_server-6] [INFO] [1780558786.099191332] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 126 X 129 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558786.102094718] [rtabmap]: rtabmap (410): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.3145s, Maps update=0.0037s pub=0.0108s (local map=32, WM=67)
[rtabmap-3] [INFO] [1780558786.156822283] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558786.157507033] [rtabmap]: rtabmap (411): Rate=0.33s, Limit=0.000s, Conversion=0.0517s, RTAB-Map=0.0019s, Maps update=0.0000s pub=0.0000s (local map=33, WM=68)
[controller_server-5] [INFO] [1780558786.195965470] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558786.312179232] [rtabmap]: rtabmap (412): Rate=0.33s, Limit=0.000s, Conversion=0.0539s, RTAB-Map=0.0932s, Maps update=0.0018s pub=0.0041s (local map=33, WM=69)
[rtabmap-3] [INFO] [1780558786.371497897] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558786.371651116] [rtabmap]: rtabmap (413): Rate=0.33s, Limit=0.000s, Conversion=0.0565s, RTAB-Map=0.0017s, Maps update=0.0000s pub=0.0000s (local map=33, WM=69)
[rtabmap-3] [INFO] [1780558786.381897402] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558786.382055389] [rtabmap]: rtabmap (414): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0096s, Maps update=0.0000s pub=0.0000s (local map=33, WM=69)
[rtabmap-3] [INFO] [1780558786.505760757] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558786.505912216] [rtabmap]: rtabmap (415): Rate=0.33s, Limit=0.000s, Conversion=0.0107s, RTAB-Map=0.0098s, Maps update=0.0000s pub=0.0000s (local map=34, WM=70)
[rtabmap-3] [INFO] [1780558786.795193458] [rtabmap]: rtabmap (416): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.2348s, Maps update=0.0008s pub=0.0021s (local map=34, WM=71)
[rtabmap-3] [INFO] [1780558786.804225031] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558786.804385803] [rtabmap]: rtabmap (417): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0082s, Maps update=0.0000s pub=0.0000s (local map=34, WM=71)
[rtabmap-3] [INFO] [1780558786.832266107] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558786.832406334] [rtabmap]: rtabmap (418): Rate=0.33s, Limit=0.000s, Conversion=0.0128s, RTAB-Map=0.0025s, Maps update=0.0000s pub=0.0000s (local map=35, WM=72)
[rtabmap-3] [INFO] [1780558786.882517294] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558786.882684561] [rtabmap]: rtabmap (419): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0039s, Maps update=0.0000s pub=0.0000s (local map=34, WM=72)
[controller_server-5] [INFO] [1780558787.196956968] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558787.310691067] [rtabmap]: rtabmap (420): Rate=0.33s, Limit=0.000s, Conversion=0.0631s, RTAB-Map=0.2922s, Maps update=0.0124s pub=0.0019s (local map=35, WM=73)
[rtabmap-3] [INFO] [1780558787.314689003] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558787.314830670] [rtabmap]: rtabmap (421): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0032s, Maps update=0.0000s pub=0.0000s (local map=36, WM=74)
[rtabmap-3] [INFO] [1780558787.573315973] [rtabmap]: rtabmap (422): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.2358s, Maps update=0.0010s pub=0.0031s (local map=36, WM=75)
[rtabmap-3] [INFO] [1780558787.611091678] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558787.611252513] [rtabmap]: rtabmap (423): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0361s, Maps update=0.0000s pub=0.0000s (local map=37, WM=76)
[rtabmap-3] [INFO] [1780558787.614455170] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558787.614827113] [rtabmap]: rtabmap (424): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0025s, Maps update=0.0000s pub=0.0000s (local map=36, WM=76)
[rtabmap-3] [INFO] [1780558787.684243903] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558787.684417827] [rtabmap]: rtabmap (425): Rate=0.33s, Limit=0.000s, Conversion=0.0106s, RTAB-Map=0.0210s, Maps update=0.0000s pub=0.0000s (local map=36, WM=76)
[rtabmap-3] [INFO] [1780558788.058884060] [rtabmap]: rtabmap (426): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.3375s, Maps update=0.0010s pub=0.0023s (local map=37, WM=77)
[rtabmap-3] [INFO] [1780558788.064031971] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558788.065718917] [rtabmap]: rtabmap (427): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0029s, Maps update=0.0000s pub=0.0000s (local map=38, WM=78)
[controller_server-5] [INFO] [1780558788.195942569] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558788.257003033] [rtabmap]: rtabmap (428): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1274s, Maps update=0.0083s pub=0.0104s (local map=38, WM=79)
[rtabmap-3] [INFO] [1780558788.277552311] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558788.277726427] [rtabmap]: rtabmap (429): Rate=0.33s, Limit=0.000s, Conversion=0.0130s, RTAB-Map=0.0051s, Maps update=0.0000s pub=0.0000s (local map=38, WM=79)
[rtabmap-3] [INFO] [1780558788.337791926] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558788.337947258] [rtabmap]: rtabmap (430): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0203s, Maps update=0.0000s pub=0.0000s (local map=38, WM=79)
[rtabmap-3] [INFO] [1780558788.385795743] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558788.385961250] [rtabmap]: rtabmap (431): Rate=0.33s, Limit=0.000s, Conversion=0.0110s, RTAB-Map=0.0087s, Maps update=0.0000s pub=0.0000s (local map=38, WM=79)
[rtabmap-3] [INFO] [1780558788.488398517] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558788.488600089] [rtabmap]: rtabmap (432): Rate=0.33s, Limit=0.000s, Conversion=0.0416s, RTAB-Map=0.0150s, Maps update=0.0000s pub=0.0000s (local map=38, WM=79)
[rtabmap-3] [INFO] [1780558788.706166558] [rtabmap]: rtabmap (433): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.1970s, Maps update=0.0007s pub=0.0001s (local map=38, WM=79)
[rtabmap-3] [INFO] [1780558788.799827968] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558788.799980323] [rtabmap]: rtabmap (434): Rate=0.33s, Limit=0.000s, Conversion=0.0416s, RTAB-Map=0.0019s, Maps update=0.0000s pub=0.0000s (local map=39, WM=80)
[rtabmap-3] [INFO] [1780558788.885213339] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558788.885394750] [rtabmap]: rtabmap (435): Rate=0.33s, Limit=0.000s, Conversion=0.0607s, RTAB-Map=0.0038s, Maps update=0.0000s pub=0.0000s (local map=38, WM=80)
[controller_server-5] [INFO] [1780558789.197702529] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558789.318414920] [rtabmap]: rtabmap (436): Rate=0.33s, Limit=0.000s, Conversion=0.0631s, RTAB-Map=0.3397s, Maps update=0.0097s pub=0.0082s (local map=39, WM=81)
[rtabmap-3] [INFO] [1780558789.815989774] [rtabmap]: rtabmap (437): Rate=0.33s, Limit=0.000s, Conversion=0.0208s, RTAB-Map=0.4455s, Maps update=0.0034s pub=0.0271s (local map=40, WM=82)
[rtabmap-3] [INFO] [1780558790.190554053] [rtabmap]: rtabmap (438): Rate=0.33s, Limit=0.000s, Conversion=0.0528s, RTAB-Map=0.2955s, Maps update=0.0025s pub=0.0214s (local map=41, WM=83)
[controller_server-5] [INFO] [1780558790.196153654] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558790.524313543] [rtabmap]: rtabmap (439): Rate=0.33s, Limit=0.000s, Conversion=0.0460s, RTAB-Map=0.2689s, Maps update=0.0021s pub=0.0103s (local map=42, WM=84)
[rtabmap-3] [INFO] [1780558790.597708597] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558790.597890521] [rtabmap]: rtabmap (440): Rate=0.33s, Limit=0.000s, Conversion=0.0537s, RTAB-Map=0.0190s, Maps update=0.0000s pub=0.0000s (local map=43, WM=85)
[rtabmap-3] [INFO] [1780558790.901137298] [rtabmap]: rtabmap (441): Rate=0.33s, Limit=0.000s, Conversion=0.0557s, RTAB-Map=0.2431s, Maps update=0.0015s pub=0.0025s (local map=43, WM=86)
[rtabmap-3] [INFO] [1780558790.925367068] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558790.931603418] [rtabmap]: rtabmap (442): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0229s, Maps update=0.0000s pub=0.0000s (local map=44, WM=87)
[rtabmap-3] [INFO] [1780558791.071342536] [rtabmap]: rtabmap (443): Rate=0.33s, Limit=0.000s, Conversion=0.0046s, RTAB-Map=0.0880s, Maps update=0.0011s pub=0.0052s (local map=44, WM=88)
[rtabmap-3] [INFO] [1780558791.196968123] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558791.197136190] [rtabmap]: rtabmap (444): Rate=0.33s, Limit=0.000s, Conversion=0.0900s, RTAB-Map=0.0068s, Maps update=0.0000s pub=0.0000s (local map=44, WM=88)
[rtabmap-3] [INFO] [1780558791.257355043] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558791.257519846] [rtabmap]: rtabmap (445): Rate=0.33s, Limit=0.000s, Conversion=0.0310s, RTAB-Map=0.0287s, Maps update=0.0000s pub=0.0000s (local map=45, WM=89)
[rtabmap-3] [INFO] [1780558791.304760708] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558791.305552596] [rtabmap]: rtabmap (446): Rate=0.33s, Limit=0.000s, Conversion=0.0410s, RTAB-Map=0.0056s, Maps update=0.0000s pub=0.0000s (local map=44, WM=89)
[rtabmap-3] [INFO] [1780558791.377825421] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558791.379828213] [rtabmap]: rtabmap (447): Rate=0.33s, Limit=0.000s, Conversion=0.0393s, RTAB-Map=0.0324s, Maps update=0.0000s pub=0.0000s (local map=44, WM=89)
[controller_server-5] [INFO] [1780558791.396001597] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558791.911373835] [rtabmap]: rtabmap (448): Rate=0.33s, Limit=0.000s, Conversion=0.0653s, RTAB-Map=0.3996s, Maps update=0.0010s pub=0.0222s (local map=45, WM=90)
[rtabmap-3] [INFO] [1780558792.159855645] [rtabmap]: rtabmap (449): Rate=0.33s, Limit=0.000s, Conversion=0.0604s, RTAB-Map=0.1723s, Maps update=0.0019s pub=0.0114s (local map=46, WM=91)
[rtabmap-3] [INFO] [1780558792.207310208] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558792.207486852] [rtabmap]: rtabmap (450): Rate=0.33s, Limit=0.000s, Conversion=0.0421s, RTAB-Map=0.0048s, Maps update=0.0000s pub=0.0000s (local map=46, WM=91)
[rtabmap-3] [INFO] [1780558792.265765731] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558792.265915718] [rtabmap]: rtabmap (451): Rate=0.33s, Limit=0.000s, Conversion=0.0512s, RTAB-Map=0.0028s, Maps update=0.0000s pub=0.0000s (local map=47, WM=92)
[controller_server-5] [INFO] [1780558792.395947893] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558792.607371226] [rtabmap]: rtabmap (452): Rate=0.33s, Limit=0.000s, Conversion=0.1048s, RTAB-Map=0.2214s, Maps update=0.0009s pub=0.0039s (local map=47, WM=93)
[rtabmap-3] [INFO] [1780558792.673810526] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558792.676905021] [rtabmap]: rtabmap (453): Rate=0.33s, Limit=0.000s, Conversion=0.0438s, RTAB-Map=0.0217s, Maps update=0.0000s pub=0.0000s (local map=48, WM=94)
[rtabmap-3] [INFO] [1780558793.067715357] [rtabmap]: rtabmap (454): Rate=0.33s, Limit=0.000s, Conversion=0.0445s, RTAB-Map=0.2801s, Maps update=0.0054s pub=0.0073s (local map=48, WM=95)
[rtabmap-3] [INFO] [1780558793.076665203] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558793.080443904] [rtabmap]: rtabmap (455): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0080s, Maps update=0.0000s pub=0.0000s (local map=49, WM=96)
[rtabmap-3] [INFO] [1780558793.369267760] [rtabmap]: rtabmap (456): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.2470s, Maps update=0.0012s pub=0.0023s (local map=49, WM=97)
[rtabmap-3] [INFO] [1780558793.374139603] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558793.376195389] [rtabmap]: rtabmap (457): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0034s, Maps update=0.0000s pub=0.0000s (local map=49, WM=97)
[rtabmap-3] [INFO] [1780558793.381123361] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558793.382886597] [rtabmap]: rtabmap (458): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0041s, Maps update=0.0000s pub=0.0000s (local map=50, WM=98)
[controller_server-5] [INFO] [1780558793.396182195] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558793.458676522] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558793.458890062] [rtabmap]: rtabmap (459): Rate=0.33s, Limit=0.000s, Conversion=0.0116s, RTAB-Map=0.0091s, Maps update=0.0000s pub=0.0000s (local map=49, WM=98)
[rtabmap-3] [INFO] [1780558793.786038378] [rtabmap]: rtabmap (460): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.2580s, Maps update=0.0011s pub=0.0042s (local map=50, WM=99)
[rtabmap-3] [INFO] [1780558793.799713504] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558793.799878723] [rtabmap]: rtabmap (461): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0128s, Maps update=0.0000s pub=0.0000s (local map=51, WM=100)
[rtabmap-3] [INFO] [1780558793.844943896] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558793.845138875] [rtabmap]: rtabmap (462): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0046s, Maps update=0.0000s pub=0.0000s (local map=50, WM=100)
[rtabmap-3] [INFO] [1780558794.178276886] [rtabmap]: rtabmap (463): Rate=0.33s, Limit=0.000s, Conversion=0.0416s, RTAB-Map=0.2229s, Maps update=0.0011s pub=0.0115s (local map=51, WM=101)
[rtabmap-3] [INFO] [1780558794.195405011] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558794.195578423] [rtabmap]: rtabmap (464): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0163s, Maps update=0.0000s pub=0.0000s (local map=51, WM=101)
[rtabmap-3] [INFO] [1780558794.365834754] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558794.366010277] [rtabmap]: rtabmap (465): Rate=0.33s, Limit=0.000s, Conversion=0.1149s, RTAB-Map=0.0327s, Maps update=0.0000s pub=0.0000s (local map=52, WM=102)
[controller_server-5] [INFO] [1780558794.395951111] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558794.585837890] [rtabmap]: rtabmap (466): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.2117s, Maps update=0.0020s pub=0.0053s (local map=52, WM=103)
[rtabmap-3] [INFO] [1780558794.657695673] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558794.657880957] [rtabmap]: rtabmap (467): Rate=0.33s, Limit=0.000s, Conversion=0.0612s, RTAB-Map=0.0076s, Maps update=0.0000s pub=0.0000s (local map=52, WM=103)
[rtabmap-3] [INFO] [1780558794.769995432] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558794.770177963] [rtabmap]: rtabmap (468): Rate=0.33s, Limit=0.000s, Conversion=0.0959s, RTAB-Map=0.0158s, Maps update=0.0000s pub=0.0000s (local map=53, WM=104)
[rtabmap-3] [INFO] [1780558795.142209191] [rtabmap]: rtabmap (469): Rate=0.33s, Limit=0.000s, Conversion=0.0207s, RTAB-Map=0.3073s, Maps update=0.0039s pub=0.0396s (local map=53, WM=105)
[rtabmap-3] [INFO] [1780558795.261548105] [rtabmap]: rtabmap (470): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1148s, Maps update=0.0012s pub=0.0025s (local map=54, WM=106)
[rtabmap-3] [INFO] [1780558795.302384074] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558795.302560366] [rtabmap]: rtabmap (471): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0238s, Maps update=0.0000s pub=0.0000s (local map=54, WM=106)
[rtabmap-3] [INFO] [1780558795.335052228] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558795.335272457] [rtabmap]: rtabmap (472): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0058s, Maps update=0.0000s pub=0.0000s (local map=54, WM=106)
[controller_server-5] [INFO] [1780558795.396114434] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558795.399439878] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558795.399599561] [rtabmap]: rtabmap (473): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0069s, Maps update=0.0000s pub=0.0000s (local map=55, WM=107)
[rtabmap-3] [INFO] [1780558795.477962599] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558795.478133482] [rtabmap]: rtabmap (474): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0200s, Maps update=0.0000s pub=0.0000s (local map=54, WM=107)
[rtabmap-3] [INFO] [1780558795.756924288] [rtabmap]: rtabmap (475): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.2108s, Maps update=0.0012s pub=0.0099s (local map=55, WM=108)
[rtabmap-3] [INFO] [1780558795.770245776] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558795.770410675] [rtabmap]: rtabmap (476): Rate=0.33s, Limit=0.000s, Conversion=0.0108s, RTAB-Map=0.0020s, Maps update=0.0000s pub=0.0000s (local map=55, WM=108)
[rtabmap-3] [INFO] [1780558795.868578405] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558795.868749257] [rtabmap]: rtabmap (477): Rate=0.33s, Limit=0.000s, Conversion=0.0662s, RTAB-Map=0.0173s, Maps update=0.0000s pub=0.0000s (local map=55, WM=108)
[rtabmap-3] [INFO] [1780558795.874638017] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558795.874792836] [rtabmap]: rtabmap (478): Rate=0.33s, Limit=0.000s, Conversion=0.0006s, RTAB-Map=0.0048s, Maps update=0.0000s pub=0.0000s (local map=56, WM=109)
[rtabmap-3] [INFO] [1780558796.146929815] [rtabmap]: rtabmap (479): Rate=0.33s, Limit=0.000s, Conversion=0.0207s, RTAB-Map=0.2023s, Maps update=0.0047s pub=0.0037s (local map=56, WM=110)
[rtabmap-3] [INFO] [1780558796.152106688] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558796.152267204] [rtabmap]: rtabmap (480): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0043s, Maps update=0.0000s pub=0.0000s (local map=56, WM=110)
[rtabmap-3] [INFO] [1780558796.180873452] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558796.181038767] [rtabmap]: rtabmap (481): Rate=0.33s, Limit=0.000s, Conversion=0.0107s, RTAB-Map=0.0020s, Maps update=0.0000s pub=0.0000s (local map=56, WM=110)
[rtabmap-3] [INFO] [1780558796.258647905] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558796.258814916] [rtabmap]: rtabmap (482): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0058s, Maps update=0.0000s pub=0.0000s (local map=56, WM=110)
[controller_server-5] [INFO] [1780558796.395994454] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558796.528826927] [rtabmap]: rtabmap (483): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1608s, Maps update=0.0019s pub=0.0023s (local map=57, WM=111)
[rtabmap-3] [INFO] [1780558796.551777860] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558796.551918407] [rtabmap]: rtabmap (484): Rate=0.33s, Limit=0.000s, Conversion=0.0119s, RTAB-Map=0.0105s, Maps update=0.0000s pub=0.0000s (local map=57, WM=111)
[rtabmap-3] [INFO] [1780558796.575079552] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558796.575240803] [rtabmap]: rtabmap (485): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0080s, Maps update=0.0000s pub=0.0000s (local map=57, WM=111)
[rtabmap-3] [INFO] [1780558796.636499591] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558796.636655146] [rtabmap]: rtabmap (486): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0021s, Maps update=0.0000s pub=0.0000s (local map=57, WM=111)
[rtabmap-3] [INFO] [1780558796.981220008] [rtabmap]: rtabmap (487): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.2058s, Maps update=0.0098s pub=0.0090s (local map=58, WM=112)
[rtabmap-3] [INFO] [1780558796.989029703] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558796.989172330] [rtabmap]: rtabmap (488): Rate=0.33s, Limit=0.000s, Conversion=0.0006s, RTAB-Map=0.0067s, Maps update=0.0000s pub=0.0000s (local map=58, WM=112)
[rtabmap-3] [INFO] [1780558797.034531049] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558797.034682221] [rtabmap]: rtabmap (489): Rate=0.33s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.0087s, Maps update=0.0000s pub=0.0000s (local map=58, WM=112)
[rtabmap-3] [INFO] [1780558797.109812910] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558797.110075123] [rtabmap]: rtabmap (490): Rate=0.33s, Limit=0.000s, Conversion=0.0115s, RTAB-Map=0.0123s, Maps update=0.0000s pub=0.0000s (local map=59, WM=113)
[rtabmap-3] [INFO] [1780558797.350567755] [rtabmap]: rtabmap (491): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1765s, Maps update=0.0029s pub=0.0187s (local map=59, WM=114)
[rtabmap-3] [INFO] [1780558797.379838593] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558797.379998053] [rtabmap]: rtabmap (492): Rate=0.33s, Limit=0.000s, Conversion=0.0249s, RTAB-Map=0.0020s, Maps update=0.0000s pub=0.0000s (local map=59, WM=114)
[controller_server-5] [INFO] [1780558797.395957739] [controller_server]: Passing new path to controller.
[planner_server-6] [INFO] [1780558797.798134894] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 127 X 129 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558797.798262449] [rtabmap]: rtabmap (493): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1835s, Maps update=0.0026s pub=0.0045s (local map=60, WM=115)
[rtabmap-3] [INFO] [1780558797.803946533] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558797.804091496] [rtabmap]: rtabmap (494): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0046s, Maps update=0.0000s pub=0.0000s (local map=60, WM=115)
[rtabmap-3] [INFO] [1780558797.813028543] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558797.813198883] [rtabmap]: rtabmap (495): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0083s, Maps update=0.0000s pub=0.0000s (local map=60, WM=115)
[rtabmap-3] [INFO] [1780558797.871269638] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558797.871439498] [rtabmap]: rtabmap (496): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0060s, Maps update=0.0000s pub=0.0000s (local map=60, WM=115)
[rtabmap-3] [INFO] [1780558797.935472968] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558797.935631371] [rtabmap]: rtabmap (497): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0043s, Maps update=0.0000s pub=0.0000s (local map=60, WM=115)
[rtabmap-3] [INFO] [1780558798.114803744] [rtabmap]: rtabmap (498): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1183s, Maps update=0.0009s pub=0.0001s (local map=60, WM=115)
[rtabmap-3] [INFO] [1780558798.125563708] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558798.125743040] [rtabmap]: rtabmap (499): Rate=0.33s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.0100s, Maps update=0.0000s pub=0.0000s (local map=60, WM=115)
[rtabmap-3] [INFO] [1780558798.136114164] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558798.136263095] [rtabmap]: rtabmap (500): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0023s, Maps update=0.0000s pub=0.0000s (local map=61, WM=116)
[rtabmap-3] [INFO] [1780558798.197567071] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558798.197790308] [rtabmap]: rtabmap (501): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0070s, Maps update=0.0000s pub=0.0000s (local map=60, WM=116)
[rtabmap-3] [INFO] [1780558798.277988207] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558798.278152530] [rtabmap]: rtabmap (502): Rate=0.33s, Limit=0.000s, Conversion=0.0210s, RTAB-Map=0.0036s, Maps update=0.0000s pub=0.0000s (local map=60, WM=116)
[rtabmap-3] [INFO] [1780558798.326965243] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558798.327130334] [rtabmap]: rtabmap (503): Rate=0.33s, Limit=0.000s, Conversion=0.0007s, RTAB-Map=0.0093s, Maps update=0.0000s pub=0.0000s (local map=60, WM=116)
[controller_server-5] [INFO] [1780558798.596690321] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558798.645983042] [rtabmap]: rtabmap (504): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.2376s, Maps update=0.0021s pub=0.0046s (local map=61, WM=117)
[rtabmap-3] [INFO] [1780558798.659599609] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558798.659759453] [rtabmap]: rtabmap (505): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0026s, Maps update=0.0000s pub=0.0000s (local map=61, WM=117)
[rtabmap-3] [INFO] [1780558798.752605963] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558798.752761710] [rtabmap]: rtabmap (506): Rate=0.33s, Limit=0.000s, Conversion=0.0316s, RTAB-Map=0.0035s, Maps update=0.0000s pub=0.0000s (local map=62, WM=118)
[planner_server-6] [INFO] [1780558798.958862541] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 128 X 129 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558798.960841621] [rtabmap]: rtabmap (507): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.1752s, Maps update=0.0020s pub=0.0050s (local map=62, WM=119)
[rtabmap-3] [INFO] [1780558798.988390858] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558798.988548237] [rtabmap]: rtabmap (508): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0266s, Maps update=0.0000s pub=0.0000s (local map=62, WM=119)
[rtabmap-3] [INFO] [1780558799.039084985] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558799.039287005] [rtabmap]: rtabmap (509): Rate=0.33s, Limit=0.000s, Conversion=0.0480s, RTAB-Map=0.0020s, Maps update=0.0000s pub=0.0000s (local map=62, WM=119)
[rtabmap-3] [INFO] [1780558799.261180806] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558799.261375402] [rtabmap]: rtabmap (510): Rate=0.33s, Limit=0.000s, Conversion=0.1631s, RTAB-Map=0.0027s, Maps update=0.0000s pub=0.0000s (local map=63, WM=120)
[controller_server-5] [INFO] [1780558799.595949372] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558799.632065985] [rtabmap]: rtabmap (511): Rate=0.33s, Limit=0.000s, Conversion=0.1157s, RTAB-Map=0.2459s, Maps update=0.0031s pub=0.0044s (local map=63, WM=121)
[rtabmap-3] [INFO] [1780558799.914847947] [rtabmap]: rtabmap (512): Rate=0.33s, Limit=0.000s, Conversion=0.0616s, RTAB-Map=0.1760s, Maps update=0.0221s pub=0.0196s (local map=64, WM=122)
[rtabmap-3] [INFO] [1780558799.956125146] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558799.956324126] [rtabmap]: rtabmap (513): Rate=0.33s, Limit=0.000s, Conversion=0.0337s, RTAB-Map=0.0045s, Maps update=0.0000s pub=0.0000s (local map=65, WM=123)
[rtabmap-3] [INFO] [1780558800.041503379] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558800.041836954] [rtabmap]: rtabmap (514): Rate=0.33s, Limit=0.000s, Conversion=0.0763s, RTAB-Map=0.0083s, Maps update=0.0000s pub=0.0000s (local map=64, WM=123)
[controller_server-5] [INFO] [1780558800.595962741] [controller_server]: Passing new path to controller.
[planner_server-6] [INFO] [1780558800.617893559] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 130 X 129 at 0.100000 m/pix
[rtabmap-3] [INFO] [1780558800.620536814] [rtabmap]: rtabmap (515): Rate=0.33s, Limit=0.000s, Conversion=0.0519s, RTAB-Map=0.4700s, Maps update=0.0040s pub=0.0444s (local map=65, WM=124)
[controller_server-5] [INFO] [1780558800.797190684] [controller_server]: Reached the goal!
[bt_navigator-8] [INFO] [1780558800.835981209] [bt_navigator]: Navigation succeeded
[frontier_explorer.py-10] [INFO] [1780558800.960211794] [frontier_explorer]: Reached frontier (-4.76842512562871, 0.1414603628218174), looking for next.
[rtabmap-3] [INFO] [1780558801.111320534] [rtabmap]: rtabmap (516): Rate=0.33s, Limit=0.000s, Conversion=0.0817s, RTAB-Map=0.3897s, Maps update=0.0080s pub=0.0093s (local map=66, WM=125)
[rtabmap-3] [INFO] [1780558801.233775151] [rtabmap]: rtabmap (517): Rate=0.33s, Limit=0.000s, Conversion=0.0492s, RTAB-Map=0.0681s, Maps update=0.0010s pub=0.0002s (local map=66, WM=125)
[rtabmap-3] [INFO] [1780558801.242194652] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558801.242361087] [rtabmap]: rtabmap (518): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0076s, Maps update=0.0000s pub=0.0000s (local map=67, WM=126)
[rtabmap-3] [INFO] [1780558801.306722124] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558801.306944945] [rtabmap]: rtabmap (519): Rate=0.33s, Limit=0.000s, Conversion=0.0615s, RTAB-Map=0.0024s, Maps update=0.0000s pub=0.0000s (local map=66, WM=126)
[cmd_vel_bridge.py-4] [INFO] [1780558801.352068881] [cmd_vel_bridge]: cmd_vel timeout, sending stop
[rtabmap-3] [INFO] [1780558801.465288715] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558801.466839147] [rtabmap]: rtabmap (520): Rate=0.33s, Limit=0.000s, Conversion=0.0126s, RTAB-Map=0.0138s, Maps update=0.0000s pub=0.0000s (local map=66, WM=126)
[rtabmap-3] [INFO] [1780558801.872220226] [rtabmap]: rtabmap (521): Rate=0.33s, Limit=0.000s, Conversion=0.0786s, RTAB-Map=0.2086s, Maps update=0.0012s pub=0.0092s (local map=67, WM=127)
[rtabmap-3] [INFO] [1780558801.903708170] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558801.904054161] [rtabmap]: rtabmap (522): Rate=0.33s, Limit=0.000s, Conversion=0.0206s, RTAB-Map=0.0103s, Maps update=0.0000s pub=0.0000s (local map=67, WM=127)
[rtabmap-3] [INFO] [1780558802.020309258] [rtabmap]: rtabmap (523): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0609s, Maps update=0.0009s pub=0.0001s (local map=67, WM=127)
[rtabmap-3] [INFO] [1780558802.107956089] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558802.108144957] [rtabmap]: rtabmap (524): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0091s, Maps update=0.0000s pub=0.0000s (local map=67, WM=127)
[rtabmap-3] [INFO] [1780558802.233073036] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558802.233249455] [rtabmap]: rtabmap (525): Rate=0.33s, Limit=0.000s, Conversion=0.0114s, RTAB-Map=0.0024s, Maps update=0.0000s pub=0.0000s (local map=68, WM=128)
[rtabmap-3] [INFO] [1780558802.304399050] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558802.304561837] [rtabmap]: rtabmap (526): Rate=0.33s, Limit=0.000s, Conversion=0.0105s, RTAB-Map=0.0133s, Maps update=0.0000s pub=0.0000s (local map=67, WM=128)
[rtabmap-3] [INFO] [1780558802.610824726] [rtabmap]: rtabmap (527): Rate=0.33s, Limit=0.000s, Conversion=0.0209s, RTAB-Map=0.2259s, Maps update=0.0086s pub=0.0041s (local map=68, WM=129)
[rtabmap-3] [INFO] [1780558802.624179209] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558802.624331372] [rtabmap]: rtabmap (528): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0121s, Maps update=0.0000s pub=0.0000s (local map=68, WM=129)
[rtabmap-3] [INFO] [1780558802.686821972] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558802.686988856] [rtabmap]: rtabmap (529): Rate=0.33s, Limit=0.000s, Conversion=0.0107s, RTAB-Map=0.0030s, Maps update=0.0000s pub=0.0000s (local map=69, WM=130)
[frontier_explorer.py-10] [INFO] [1780558802.712798828] [frontier_explorer]: Selected frontier: (-6.57,0.64) score=6.61 gain=428 size=51 d=2.46m turn=24deg clear=0.32m from 17 candidates
[bt_navigator-8] [INFO] [1780558802.730291348] [bt_navigator]: Begin navigating from current location to (-6.57, 0.64)
[controller_server-5] [INFO] [1780558802.780141208] [controller_server]: Received a goal, begin computing control effort.
[controller_server-5] [INFO] [1780558802.984044610] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558803.088753651] [rtabmap]: rtabmap (530): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.3241s, Maps update=0.0039s pub=0.0160s (local map=69, WM=131)
[rtabmap-3] [INFO] [1780558803.141062251] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558803.141187053] [rtabmap]: rtabmap (531): Rate=0.33s, Limit=0.000s, Conversion=0.0392s, RTAB-Map=0.0126s, Maps update=0.0000s pub=0.0000s (local map=69, WM=131)
[rtabmap-3] [INFO] [1780558803.538801021] [rtabmap]: rtabmap (532): Rate=0.33s, Limit=0.000s, Conversion=0.0118s, RTAB-Map=0.2545s, Maps update=0.0021s pub=0.0125s (local map=70, WM=132)
[rtabmap-3] [INFO] [1780558803.573173058] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558803.573367398] [rtabmap]: rtabmap (533): Rate=0.33s, Limit=0.000s, Conversion=0.0225s, RTAB-Map=0.0112s, Maps update=0.0000s pub=0.0000s (local map=71, WM=133)
[rtabmap-3] [INFO] [1780558803.628135313] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558803.628325781] [rtabmap]: rtabmap (534): Rate=0.33s, Limit=0.000s, Conversion=0.0343s, RTAB-Map=0.0097s, Maps update=0.0000s pub=0.0000s (local map=70, WM=133)
[rtabmap-3] [INFO] [1780558803.973927283] [rtabmap]: rtabmap (535): Rate=0.33s, Limit=0.000s, Conversion=0.0108s, RTAB-Map=0.1751s, Maps update=0.0030s pub=0.0025s (local map=71, WM=134)
[rtabmap-3] [INFO] [1780558803.977383066] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558803.977533565] [rtabmap]: rtabmap (536): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.0023s, Maps update=0.0000s pub=0.0000s (local map=72, WM=135)
[controller_server-5] [INFO] [1780558803.983487544] [controller_server]: Passing new path to controller.
[rtabmap-3] [INFO] [1780558804.022040820] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558804.022228472] [rtabmap]: rtabmap (537): Rate=0.33s, Limit=0.000s, Conversion=0.0323s, RTAB-Map=0.0104s, Maps update=0.0000s pub=0.0000s (local map=71, WM=135)
[rtabmap-3] [INFO] [1780558804.129810696] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558804.129990572] [rtabmap]: rtabmap (538): Rate=0.33s, Limit=0.000s, Conversion=0.0726s, RTAB-Map=0.0131s, Maps update=0.0000s pub=0.0000s (local map=72, WM=136)
[rtabmap-3] [INFO] [1780558804.165298150] [rtabmap]: Intermediate node added
[rtabmap-3] [INFO] [1780558804.165461641] [rtabmap]: rtabmap (539): Rate=0.33s, Limit=0.000s, Conversion=0.0313s, RTAB-Map=0.0035s, Maps update=0.0000s pub=0.0000s (local map=71, WM=136)
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[lifecycle_manager-9] [INFO] [1780558804.391095746] [rclcpp]: signal_handler(signal_value=2)
[INFO] [planner_server-6]: process has finished cleanly [pid 152814]
[INFO] [recoveries_server-7]: process has finished cleanly [pid 152816]
[bt_navigator-8] [INFO] [1780558804.391246981] [rclcpp]: signal_handler(signal_value=2)
[recoveries_server-7] [INFO] [1780558804.391341159] [rclcpp]: signal_handler(signal_value=2)
[INFO] [controller_server-5]: process has finished cleanly [pid 152812]
[planner_server-6] [INFO] [1780558804.391434121] [rclcpp]: signal_handler(signal_value=2)
[bt_navigator-8] [ERROR] [1780558804.402794452] [bt_navigator]: Action server failed while executing action callback: "Failed to create interrupt guard condition in Executor constructor: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called., at /tmp/binarydeb/ros-foxy-rcl-1.1.14/src/rcl/guard_condition.c:67"
[bt_navigator-8] [WARN] [1780558804.402850261] [bt_navigator]: [navigate_to_pose] [ActionServer] Aborting handle.
[bt_navigator-8] [INFO] [1780558804.426131190] [bt_navigator]: Destroying
[rtabmap-3] [INFO] [1780558804.392091862] [rclcpp]: signal_handler(signal_value=2)
[controller_server-5] [INFO] [1780558804.402439244] [rclcpp]: signal_handler(signal_value=2)
[controller_server-5] [ERROR] [1780558804.419216583] [controller_server_rclcpp_node]: Action server failed while executing action callback: "could not count subscribers: rcl node's context is invalid, at /tmp/binarydeb/ros-foxy-rcl-1.1.14/src/rcl/node.c:441"
[controller_server-5] [WARN] [1780558804.419260328] [controller_server_rclcpp_node]: [follow_path] [ActionServer] Aborting handle.
[controller_server-5] 
[controller_server-5] >>> [rcutils|error_handling.c:108] rcutils_set_error_state()
[controller_server-5] This error state is being overwritten:
[controller_server-5] 
[controller_server-5]   'rcl node's context is invalid, at /tmp/binarydeb/ros-foxy-rcl-1.1.14/src/rcl/node.c:441'
[controller_server-5] 
[controller_server-5] with this new error message:
[controller_server-5] 
[controller_server-5]   'publisher's context is invalid, at /tmp/binarydeb/ros-foxy-rcl-1.1.14/src/rcl/publisher.c:423'
[controller_server-5] 
[controller_server-5] rcutils_reset_error() should be called after error handling to avoid this.
[controller_server-5] <<<
[lifecycle_manager-9] [INFO] [1780558804.417436802] [lifecycle_manager_navigation]: Destroying
[bt_navigator-8] 
[bt_navigator-8] >>> [rcutils|error_handling.c:108] rcutils_set_error_state()
[bt_navigator-8] This error state is being overwritten:
[bt_navigator-8] 
[bt_navigator-8]   'invalid data size, at /home/unitree/cyclonedds_ws/src/ros2/rmw_cyclonedds/rmw_cyclonedds_cpp/src/serdata.cpp:308'
[bt_navigator-8] 
[bt_navigator-8] with this new error message:
[bt_navigator-8] 
[bt_navigator-8]   'the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called., at /tmp/binarydeb/ros-foxy-rcl-1.1.14/src/rcl/guard_condition.c:67'
[bt_navigator-8] 
[bt_navigator-8] rcutils_reset_error() should be called after error handling to avoid this.
[bt_navigator-8] <<<
[bt_navigator-8] terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
[bt_navigator-8]   what():  Failed to create interrupt guard condition in Executor constructor: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called., at /tmp/binarydeb/ros-foxy-rcl-1.1.14/src/rcl/guard_condition.c:67
[rtabmap-3] [INFO] [1780558804.498663217] [rtabmap]: rtabmap (540): Rate=0.33s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=0.2512s, Maps update=0.0002s pub=0.0013s (local map=72, WM=137)
[rtabmap-3] [INFO] [1780558804.543948634] [rtabmap]: Parameters are not saved (No configuration file provided...)
[INFO] [lifecycle_manager-9]: process has finished cleanly [pid 152820]
[frontier_explorer.py-10] Exception ignored in: <function ActionClient.__del__ at 0xffff798da670>
[frontier_explorer.py-10] Traceback (most recent call last):
[frontier_explorer.py-10]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/action/client.py", line 605, in __del__
[frontier_explorer.py-10]     self.destroy()
[frontier_explorer.py-10]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/action/client.py", line 598, in destroy
[frontier_explorer.py-10]     with self._node.handle as node_capsule:
[frontier_explorer.py-10]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/handle.py", line 124, in __enter__
[frontier_explorer.py-10]     return self._get_capsule()
[frontier_explorer.py-10]   File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/handle.py", line 106, in _get_capsule
[frontier_explorer.py-10]     raise InvalidHandle('Tried to use a handle that has been destroyed.')
[frontier_explorer.py-10] rclpy.handle.InvalidHandle: Tried to use a handle that has been destroyed.
[INFO] [cmd_vel_bridge.py-4]: process has finished cleanly [pid 152638]
[ERROR] [bt_navigator-8]: process has died [pid 152818, exit code -6, cmd '/opt/ros/foxy/lib/nav2_bt_navigator/bt_navigator --ros-args -r __node:=bt_navigator --params-file /home/unitree/nuwax-robot-bridge/services/geometry/ros2_ws/src/GO2_RTABMAP/install/go2_nav/share/go2_nav/config/nav2_params.yaml --params-file /tmp/launch_params_nge0qein'].
[INFO] [cloud_to_scan.py-2]: process has finished cleanly [pid 152634]
[INFO] [frontier_explorer.py-10]: process has finished cleanly [pid 153091]
[INFO] [odom_tf_broadcaster.py-1]: process has finished cleanly [pid 152632]
[INFO] [rtabmap-3]: process has finished cleanly [pid 152636]
