[INFO] [launch]: All log files can be found below /home/unitree/.ros/log/2026-05-22-17-13-09-194625-ubuntu-254545
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [python3-1]: process started with pid [254677]
[INFO] [static_transform_publisher-2]: process started with pid [254679]
[INFO] [rtabmap-3]: process started with pid [254681]
[INFO] [python3-4]: process started with pid [254683]
[INFO] [python3-5]: process started with pid [254693]
[static_transform_publisher-2] [INFO] [1779441189.620449024] [base_to_lidar_tf]: Spinning until killed publishing transform from 'base_link' to 'utlidar_lidar'
[python3-5] [INFO] [1779441193.507733062] [map_odom_publisher]: Publishing corrected odom on /rtabmap/localization_odom: map -> base_link
[python3-4] [INFO] [1779441194.069687707] [auto_switch_to_mapping]: Waiting for RTAB-Map relocalization before switching to mapping (info_topic=/info, service=/rtabmap/set_mode_mapping)
[rtabmap-3] [INFO] [1779441195.253026045] [rtabmap]: rtabmap(maps): map_filter_radius          = 0.000000
[rtabmap-3] [INFO] [1779441195.263523064] [rtabmap]: rtabmap(maps): map_filter_angle           = 30.000000
[rtabmap-3] [INFO] [1779441195.266304231] [rtabmap]: rtabmap(maps): map_cleanup                = true
[rtabmap-3] [INFO] [1779441195.273227115] [rtabmap]: rtabmap(maps): map_always_update          = false
[rtabmap-3] [INFO] [1779441195.275769070] [rtabmap]: rtabmap(maps): map_empty_ray_tracing      = true
[rtabmap-3] [INFO] [1779441195.277042191] [rtabmap]: rtabmap(maps): cloud_output_voxelized     = true
[rtabmap-3] [INFO] [1779441195.277087122] [rtabmap]: rtabmap(maps): cloud_subtract_filtering   = false
[rtabmap-3] [INFO] [1779441195.277112339] [rtabmap]: rtabmap(maps): cloud_subtract_filtering_min_neighbors = 2
[rtabmap-3] [INFO] [1779441195.282895676] [rtabmap]: rtabmap(maps): octomap_tree_depth         = 16
[rtabmap-3] [INFO] [1779441195.905838203] [rtabmap]: rtabmap: frame_id      = base_link
[rtabmap-3] [INFO] [1779441195.908420640] [rtabmap]: rtabmap: odom_frame_id = odom
[rtabmap-3] [INFO] [1779441195.910586192] [rtabmap]: rtabmap: map_frame_id  = map
[rtabmap-3] [INFO] [1779441195.911787277] [rtabmap]: rtabmap: log_to_rosout_level  = 4
[rtabmap-3] [INFO] [1779441195.912715901] [rtabmap]: rtabmap: initial_pose  = 
[rtabmap-3] [INFO] [1779441195.920723673] [rtabmap]: rtabmap: use_action_for_goal  = false
[rtabmap-3] [INFO] [1779441195.922902185] [rtabmap]: rtabmap: tf_delay      = 0.050000
[rtabmap-3] [INFO] [1779441195.924496795] [rtabmap]: rtabmap: tf_tolerance  = 1.000000
[rtabmap-3] [INFO] [1779441195.925786045] [rtabmap]: rtabmap: odom_sensor_sync   = false
[rtabmap-3] [INFO] [1779441195.927055326] [rtabmap]: rtabmap: gen_scan  = false
[rtabmap-3] [INFO] [1779441195.928288222] [rtabmap]: rtabmap: gen_depth  = false
[rtabmap-3] [INFO] [1779441195.928373954] [rtabmap]: rtabmap: scan_cloud_max_points = 0
[rtabmap-3] [INFO] [1779441195.928440198] [rtabmap]: rtabmap: scan_cloud_is_2d = false
[rtabmap-3] [INFO] [1779441196.673812366] [rtabmap]: Setting RTAB-Map parameter "Grid/MaxGroundHeight"="0.12"
[rtabmap-3] [INFO] [1779441196.677644403] [rtabmap]: Setting RTAB-Map parameter "Grid/MaxObstacleHeight"="2.5"
[rtabmap-3] [INFO] [1779441196.682249824] [rtabmap]: Setting RTAB-Map parameter "Grid/MinClusterSize"="1"
[rtabmap-3] [INFO] [1779441196.690781910] [rtabmap]: Setting RTAB-Map parameter "Grid/MinGroundHeight"="-0.6"
[rtabmap-3] [INFO] [1779441196.693870709] [rtabmap]: Setting RTAB-Map parameter "Grid/NoiseFilteringMinNeighbors"="0"
[rtabmap-3] [INFO] [1779441196.704691969] [rtabmap]: Setting RTAB-Map parameter "Grid/PreVoxelFiltering"="false"
[rtabmap-3] [INFO] [1779441196.707946728] [rtabmap]: Setting RTAB-Map parameter "Grid/RangeMax"="15.0"
[rtabmap-3] [INFO] [1779441196.711318518] [rtabmap]: Setting RTAB-Map parameter "Grid/RangeMin"="0.2"
[rtabmap-3] [INFO] [1779441196.714598238] [rtabmap]: Setting RTAB-Map parameter "Grid/RayTracing"="true"
[rtabmap-3] [INFO] [1779441196.724041796] [rtabmap]: Setting RTAB-Map parameter "Grid/Sensor"="0"
[rtabmap-3] [INFO] [1779441196.769127473] [rtabmap]: Setting RTAB-Map parameter "GridGlobal/OccupancyThr"="0.45"
[rtabmap-3] [INFO] [1779441196.778960010] [rtabmap]: Setting RTAB-Map parameter "GridGlobal/ProbClampingMax"="0.96"
[rtabmap-3] [INFO] [1779441196.784254042] [rtabmap]: Setting RTAB-Map parameter "GridGlobal/ProbClampingMin"="0.1"
[rtabmap-3] [INFO] [1779441196.790447609] [rtabmap]: Setting RTAB-Map parameter "GridGlobal/ProbHit"="0.82"
[rtabmap-3] [INFO] [1779441196.794813401] [rtabmap]: Setting RTAB-Map parameter "GridGlobal/ProbMiss"="0.47"
[rtabmap-3] [INFO] [1779441196.826759811] [rtabmap]: Setting RTAB-Map parameter "Icp/CorrespondenceRatio"="0.25"
[rtabmap-3] [INFO] [1779441196.845420930] [rtabmap]: Setting RTAB-Map parameter "Icp/Iterations"="20"
[rtabmap-3] [INFO] [1779441196.854519990] [rtabmap]: Setting RTAB-Map parameter "Icp/MaxCorrespondenceDistance"="0.2"
[rtabmap-3] [INFO] [1779441196.859999280] [rtabmap]: Setting RTAB-Map parameter "Icp/MaxRotation"="0.7"
[rtabmap-3] [INFO] [1779441196.864581083] [rtabmap]: Setting RTAB-Map parameter "Icp/MaxTranslation"="0.5"
[rtabmap-3] [INFO] [1779441197.193630856] [rtabmap]: Setting RTAB-Map parameter "Mem/IncrementalMemory"="false"
[rtabmap-3] [INFO] [1779441197.198311928] [rtabmap]: Setting RTAB-Map parameter "Mem/InitWMWithAllNodes"="true"
[rtabmap-3] [INFO] [1779441197.230970118] [rtabmap]: Setting RTAB-Map parameter "Mem/NotLinkedNodesKept"="false"
[rtabmap-3] [INFO] [1779441197.252260332] [rtabmap]: Setting RTAB-Map parameter "Mem/STMSize"="30"
[rtabmap-3] [INFO] [1779441197.343776330] [rtabmap]: Setting RTAB-Map parameter "RGBD/AngularUpdate"="0.05"
[rtabmap-3] [INFO] [1779441197.346597147] [rtabmap]: Setting RTAB-Map parameter "RGBD/LinearUpdate"="0.05"
[rtabmap-3] [INFO] [1779441197.361682146] [rtabmap]: Setting RTAB-Map parameter "RGBD/MaxOdomCacheSize"="0"
[rtabmap-3] [INFO] [1779441197.364117919] [rtabmap]: Setting RTAB-Map parameter "RGBD/NeighborLinkRefining"="true"
[rtabmap-3] [INFO] [1779441197.368609990] [rtabmap]: Setting RTAB-Map parameter "RGBD/OptimizeMaxError"="20.0"
[rtabmap-3] [INFO] [1779441197.400879712] [rtabmap]: Setting RTAB-Map parameter "RGBD/ProximityGlobalScanMap"="true"
[rtabmap-3] [INFO] [1779441197.408142357] [rtabmap]: Setting RTAB-Map parameter "RGBD/ProximityOdomGuess"="true"
[rtabmap-3] [INFO] [1779441197.414638403] [rtabmap]: Setting RTAB-Map parameter "RGBD/ProximityPathMaxNeighbors"="10"
[rtabmap-3] [INFO] [1779441197.420957031] [rtabmap]: Setting RTAB-Map parameter "Reg/Force3DoF"="true"
[rtabmap-3] [INFO] [1779441197.424371095] [rtabmap]: Setting RTAB-Map parameter "Reg/Strategy"="1"
[rtabmap-3] [INFO] [1779441197.427398226] [rtabmap]: Setting RTAB-Map parameter "Rtabmap/CreateIntermediateNodes"="true"
[rtabmap-3] [INFO] [1779441197.428543629] [rtabmap]: Setting RTAB-Map parameter "Rtabmap/DetectionRate"="8.0"
[rtabmap-3] [INFO] [1779441197.581940022] [rtabmap]: Setting "Icp/PointToPlaneRadius" parameter to 0 (default 0.000000) as "subscribe_scan_cloud" is true.
[rtabmap-3] [INFO] [1779441197.585400936] [rtabmap]: Update RTAB-Map parameter "Kp/MaxFeatures"="-1" from database
[rtabmap-3] [INFO] [1779441197.586160559] [rtabmap]: RTAB-Map detection rate = 8.000000 Hz
[rtabmap-3] [INFO] [1779441197.586318295] [rtabmap]: Create intermediate nodes
[rtabmap-3] [INFO] [1779441197.586513825] [rtabmap]: rtabmap: Using database from "/home/unitree/nuwax-robot-bridge/runtime_data/geometry/rtabmap/my_new_map.db" (6 MB).
[rtabmap-3] [INFO] [1779441198.418154656] [rtabmap]: rtabmap: 2D occupancy grid map loaded (217x324).
[rtabmap-3] [INFO] [1779441200.389778949] [rtabmap]: rtabmap: Working Memory = 76, Local map = 75.
[rtabmap-3] [INFO] [1779441200.390281215] [rtabmap]: rtabmap: Database version = "0.21.1".
[rtabmap-3] [INFO] [1779441200.390377796] [rtabmap]: rtabmap: Localization mode (Mem/IncrementalMemory=false)
[rtabmap-3] [INFO] [1779441201.334619715] [rtabmap]: Setup callbacks
[rtabmap-3] [INFO] [1779441201.336766578] [rtabmap]: rtabmap: subscribe_depth = false
[rtabmap-3] [INFO] [1779441201.336911769] [rtabmap]: rtabmap: subscribe_rgb = false
[rtabmap-3] [INFO] [1779441201.336974172] [rtabmap]: rtabmap: subscribe_stereo = false
[rtabmap-3] [INFO] [1779441201.337026527] [rtabmap]: rtabmap: subscribe_rgbd = false (rgbd_cameras=1)
[rtabmap-3] [INFO] [1779441201.337078242] [rtabmap]: rtabmap: subscribe_odom_info = false
[rtabmap-3] [INFO] [1779441201.337136357] [rtabmap]: rtabmap: subscribe_user_data = false
[rtabmap-3] [INFO] [1779441201.337189511] [rtabmap]: rtabmap: subscribe_scan = false
[rtabmap-3] [INFO] [1779441201.337237578] [rtabmap]: rtabmap: subscribe_scan_cloud = true
[rtabmap-3] [INFO] [1779441201.338702325] [rtabmap]: rtabmap: subscribe_scan_descriptor = false
[rtabmap-3] [INFO] [1779441201.338803418] [rtabmap]: rtabmap: queue_size      = 10
[rtabmap-3] [INFO] [1779441201.339140235] [rtabmap]: rtabmap: qos_image       = 0
[rtabmap-3] [INFO] [1779441201.339213615] [rtabmap]: rtabmap: qos_camera_info = 0
[rtabmap-3] [INFO] [1779441201.339280755] [rtabmap]: rtabmap: qos_scan        = 0
[rtabmap-3] [INFO] [1779441201.339346230] [rtabmap]: rtabmap: qos_odom        = 0
[rtabmap-3] [INFO] [1779441201.339416409] [rtabmap]: rtabmap: qos_user_data   = 0
[rtabmap-3] [INFO] [1779441201.341024364] [rtabmap]: rtabmap: approx_sync     = false
[rtabmap-3] [INFO] [1779441201.341152627] [rtabmap]: Setup scan callback
[rtabmap-3] [INFO] [1779441201.361755027] [rtabmap]: 
[rtabmap-3] rtabmap subscribed to:
[rtabmap-3]    /utlidar/cloud_base
[rtabmap-3] [WARN] [1779441201.361988895] [rtabmap]: There is no image subscription, bag-of-words loop closure detection will be disabled...
[rtabmap-3] [INFO] [1779441204.673433759] [rtabmap]: rtabmap (86): Rate=0.12s, Limit=0.000s, Conversion=0.0005s, RTAB-Map=2.1077s, Maps update=0.0010s pub=0.0749s (local map=75, WM=76)
[rtabmap-3] [WARN] [1779441204.676937075] [rtabmap]: New publisher discovered on this topic, offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
[rtabmap-3] [INFO] [1779441206.588976529] [rtabmap]: rtabmap (87): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.9072s, Maps update=0.0008s pub=0.0012s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441207.777400745] [rtabmap]: rtabmap (88): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.1861s, Maps update=0.0007s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441209.080100601] [rtabmap]: rtabmap (89): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2979s, Maps update=0.0007s pub=0.0034s (local map=75, WM=76)
[python3-4] [INFO] [1779441209.087857094] [auto_switch_to_mapping]: RTAB-Map relocalization match detected (loop_closure_id=0, proximity_detection_id=75, matches=1/1)
[rtabmap-3] [INFO] [1779441211.063095255] [rtabmap]: rtabmap (90): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.9774s, Maps update=0.0011s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441212.954043978] [rtabmap]: rtabmap (91): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.8887s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[python3-4] [ERROR] [1779441214.135085712] [auto_switch_to_mapping]: Mapping mode service is not available.
[rtabmap-3] [INFO] [1779441214.748765126] [rtabmap]: rtabmap (92): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.7927s, Maps update=0.0007s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441216.473707996] [rtabmap]: rtabmap (93): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.7120s, Maps update=0.0008s pub=0.0115s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441217.568612873] [rtabmap]: rtabmap (94): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.0926s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441219.290095920] [rtabmap]: rtabmap (95): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.7170s, Maps update=0.0009s pub=0.0029s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441220.517608703] [rtabmap]: rtabmap (96): Rate=0.12s, Limit=0.000s, Conversion=0.0026s, RTAB-Map=1.2229s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441221.691092180] [rtabmap]: rtabmap (97): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.1712s, Maps update=0.0010s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441222.991231482] [rtabmap]: rtabmap (98): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2974s, Maps update=0.0010s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441223.994749396] [rtabmap]: rtabmap (99): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.0013s, Maps update=0.0009s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441225.418534887] [rtabmap]: rtabmap (100): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.4216s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441226.727361798] [rtabmap]: rtabmap (101): Rate=0.12s, Limit=0.000s, Conversion=0.0001s, RTAB-Map=1.3046s, Maps update=0.0009s pub=0.0026s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441227.800834071] [rtabmap]: rtabmap (102): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.0711s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441229.227187492] [rtabmap]: rtabmap (103): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.4230s, Maps update=0.0012s pub=0.0011s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441230.990827773] [rtabmap]: rtabmap (104): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.7612s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441233.212921172] [rtabmap]: rtabmap (105): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.2074s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441234.486835748] [rtabmap]: rtabmap (106): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2693s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441235.802124453] [rtabmap]: rtabmap (107): Rate=0.12s, Limit=0.000s, Conversion=0.0001s, RTAB-Map=1.3097s, Maps update=0.0042s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441237.766201047] [rtabmap]: rtabmap (108): Rate=0.12s, Limit=0.000s, Conversion=0.0029s, RTAB-Map=1.9590s, Maps update=0.0008s pub=0.0007s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441238.805513093] [rtabmap]: rtabmap (109): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.0370s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441240.035130449] [rtabmap]: rtabmap (110): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2276s, Maps update=0.0008s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441240.609487757] [rtabmap]: rtabmap (111): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.5551s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441241.831993543] [rtabmap]: rtabmap (112): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2202s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441243.603590662] [rtabmap]: rtabmap (113): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.7694s, Maps update=0.0009s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441244.651045211] [rtabmap]: rtabmap (114): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.0393s, Maps update=0.0019s pub=0.0032s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441245.811094378] [rtabmap]: rtabmap (115): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.1353s, Maps update=0.0227s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441247.193555754] [rtabmap]: rtabmap (116): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.3559s, Maps update=0.0013s pub=0.0061s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441247.919973354] [rtabmap]: rtabmap (117): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.7244s, Maps update=0.0007s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441249.378941919] [rtabmap]: rtabmap (118): Rate=0.12s, Limit=0.000s, Conversion=0.0001s, RTAB-Map=1.4569s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441250.042644365] [rtabmap]: rtabmap (119): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.6616s, Maps update=0.0009s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441250.522803064] [rtabmap]: rtabmap (120): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.4761s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441251.768561248] [rtabmap]: rtabmap (121): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2429s, Maps update=0.0009s pub=0.0012s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441253.362448846] [rtabmap]: rtabmap (122): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.5905s, Maps update=0.0008s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441253.923090413] [rtabmap]: rtabmap (123): Rate=0.12s, Limit=0.000s, Conversion=0.0001s, RTAB-Map=0.5586s, Maps update=0.0008s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441255.167350202] [rtabmap]: rtabmap (124): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2392s, Maps update=0.0009s pub=0.0007s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441256.530735659] [rtabmap]: rtabmap (125): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.3274s, Maps update=0.0008s pub=0.0056s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441258.020557063] [rtabmap]: rtabmap (126): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.4828s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441259.689239845] [rtabmap]: rtabmap (127): Rate=0.12s, Limit=0.000s, Conversion=0.0031s, RTAB-Map=1.6456s, Maps update=0.0065s pub=0.0130s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441260.093039047] [rtabmap]: rtabmap (128): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.4014s, Maps update=0.0011s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441261.394651491] [rtabmap]: rtabmap (129): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2981s, Maps update=0.0010s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441262.694814584] [rtabmap]: rtabmap (130): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2978s, Maps update=0.0008s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441264.238764835] [rtabmap]: rtabmap (131): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.5416s, Maps update=0.0010s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441265.492570564] [rtabmap]: rtabmap (132): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2438s, Maps update=0.0008s pub=0.0007s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441266.518870031] [rtabmap]: rtabmap (133): Rate=0.12s, Limit=0.000s, Conversion=0.0029s, RTAB-Map=1.0214s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441267.911658919] [rtabmap]: rtabmap (134): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.3859s, Maps update=0.0051s pub=0.0011s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441269.054846459] [rtabmap]: rtabmap (135): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.1383s, Maps update=0.0009s pub=0.0024s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441270.401588979] [rtabmap]: rtabmap (136): Rate=0.12s, Limit=0.000s, Conversion=0.0013s, RTAB-Map=1.3328s, Maps update=0.0007s pub=0.0018s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441272.161052340] [rtabmap]: rtabmap (137): Rate=0.12s, Limit=0.000s, Conversion=0.0034s, RTAB-Map=1.7517s, Maps update=0.0008s pub=0.0028s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441274.268525238] [rtabmap]: rtabmap (138): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.9800s, Maps update=0.0048s pub=0.1219s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441275.963481773] [rtabmap]: rtabmap (139): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.6924s, Maps update=0.0010s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441278.370605078] [rtabmap]: rtabmap (140): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.2895s, Maps update=0.0009s pub=0.0008s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441279.642784111] [rtabmap]: rtabmap (141): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.2697s, Maps update=0.0010s pub=0.0008s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441280.836958273] [rtabmap]: rtabmap (142): Rate=0.12s, Limit=0.000s, Conversion=0.0001s, RTAB-Map=1.1921s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441282.653466354] [rtabmap]: rtabmap (143): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.8141s, Maps update=0.0007s pub=0.0009s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441284.061768672] [rtabmap]: rtabmap (144): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.4063s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441285.912524434] [rtabmap]: rtabmap (145): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.8476s, Maps update=0.0009s pub=0.0015s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441288.074154266] [rtabmap]: rtabmap (146): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.1593s, Maps update=0.0011s pub=0.0005s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441290.271041318] [rtabmap]: rtabmap (147): Rate=0.12s, Limit=0.000s, Conversion=0.0001s, RTAB-Map=2.1946s, Maps update=0.0009s pub=0.0007s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441293.156352087] [rtabmap]: rtabmap (148): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.8832s, Maps update=0.0007s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441294.019439640] [rtabmap]: rtabmap (149): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.8609s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441294.867548485] [rtabmap]: rtabmap (150): Rate=0.12s, Limit=0.000s, Conversion=0.0001s, RTAB-Map=0.8455s, Maps update=0.0009s pub=0.0010s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441297.584392095] [rtabmap]: rtabmap (151): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.7116s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441299.587640316] [rtabmap]: rtabmap (152): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.9656s, Maps update=0.0008s pub=0.0361s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441301.861206473] [rtabmap]: rtabmap (153): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.2706s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441303.930965452] [rtabmap]: rtabmap (154): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.0675s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441306.341751013] [rtabmap]: rtabmap (155): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.3988s, Maps update=0.0022s pub=0.0028s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441308.721449283] [rtabmap]: rtabmap (156): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.3776s, Maps update=0.0007s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441310.646440537] [rtabmap]: rtabmap (157): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.9226s, Maps update=0.0009s pub=0.0008s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441311.242076002] [rtabmap]: rtabmap (158): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.5934s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441313.382920333] [rtabmap]: rtabmap (159): Rate=0.12s, Limit=0.000s, Conversion=0.0003s, RTAB-Map=2.1376s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441315.678982147] [rtabmap]: rtabmap (160): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.2939s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441318.083343087] [rtabmap]: rtabmap (161): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.4015s, Maps update=0.0014s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441320.044709690] [rtabmap]: rtabmap (162): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.9488s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441321.630708197] [rtabmap]: rtabmap (163): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.5838s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441322.657543315] [rtabmap]: rtabmap (164): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.9601s, Maps update=0.0008s pub=0.0007s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441324.896124395] [rtabmap]: rtabmap (165): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.2278s, Maps update=0.0021s pub=0.0060s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441326.991308798] [rtabmap]: rtabmap (166): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.0836s, Maps update=0.0009s pub=0.0008s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441329.337113469] [rtabmap]: rtabmap (167): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.3396s, Maps update=0.0008s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441331.523705460] [rtabmap]: rtabmap (168): Rate=0.12s, Limit=0.000s, Conversion=0.0007s, RTAB-Map=2.1637s, Maps update=0.0040s pub=0.0010s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441333.485734039] [rtabmap]: rtabmap (169): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.9594s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441336.479360850] [rtabmap]: rtabmap (170): Rate=0.12s, Limit=0.000s, Conversion=0.0001s, RTAB-Map=2.9912s, Maps update=0.0009s pub=0.0009s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441338.551209712] [rtabmap]: rtabmap (171): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.0695s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441340.267270326] [rtabmap]: rtabmap (172): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=1.7137s, Maps update=0.0009s pub=0.0006s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441340.941506848] [rtabmap]: rtabmap (173): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=0.6454s, Maps update=0.0009s pub=0.0271s (local map=75, WM=76)
[rtabmap-3] [INFO] [1779441343.010710188] [rtabmap]: rtabmap (174): Rate=0.12s, Limit=0.000s, Conversion=0.0002s, RTAB-Map=2.0667s, Maps update=0.0011s pub=0.0005s (local map=75, WM=76)
