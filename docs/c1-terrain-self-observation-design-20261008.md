# 地形、自体观测与净空：正在验证的改造

2026-10-08；这是候选设计，不是实机验收结论。

用户反馈平整瓷砖上的足端回波污染地面，同时要求真实起伏和草地能力。九月离线实录并非已确认的此次瓷砖采集。当前实现已有坡阶和多高度带，不应概括为只支持共面地面；本机区分场景仍发现密集起伏的支撑连段失败。

## 已定位问题

- 上游自体过滤使用静态全机凸柱并保留低支撑带，可能留下低足端回波，又误删腿间真实障碍。
- 导航以机体整个投影逐列连续支撑近似可通行，不能等同于实际逐足接触能力。
- 查询端自体剔除、支撑带生成及派生地图重建耦合，取消当前姿态有效早退会同时启用这些行为。两版定点改善但完整路线退化的候选均未准入。
- 回波高度没有直接提供自体、植被或承重地面的语义证明，不能把所有低点清空或抹平成名义地面。

## 固定实体阶段（离线验证通过，未做实机验收）

保留导航footprint/z字段，另从选定模型基准的fixed-joint连通分量提取collision盒、球、圆柱并集。排除活动关节子树，不用visual、mesh外包、关节零位或手填全机尺寸冒充精确实体。原始输入mask与注册点云使用同一实体内部判定。

被删除回波不生成自由空间、不生成支撑，也不会修复旧污染地图。记录模型连杆对应runtime body的既有配置声明，不以名称相等新增门槛，也不声称日志证明物理标定。

完整手填尺寸继续按现有导航配置优先级覆盖模型，固定实体为零时过滤停用并输出原因。没有可信固定实体不会回退到导航凸包。活动腿回波可能比旧凸包保留更多；这一阶段不能声称动态足端问题解决。

动态足端需要实测关节映射、源时间与LiDAR时钟关系、覆盖扫描的关节历史以及采样时刻FK。命令角、最新getter或参考姿态都不能替代这些数据。

## 导航阶段（独立进行）

拆分自体观测证据、碰撞几何、支撑能力。新增能力必须由生产查询消费；缺少逐足输入时不能只加一个闲置开关就宣称有足步规划。优先用已有数据修正粗糙地面关联和不合理的全柱/逐格假设，保留硬障碍、低顶、坑洞和多层反例。此前5厘米约定不能扩展为所有高度判断的统一容差。

## 参考资料

- [robot_body_filter](https://github.com/ctu-vras/robot_body_filter)：逐连杆碰撞与采样姿态过滤，参考模型分离思路，不强制引入ROS依赖。
- [elevation_mapping](https://github.com/ANYbotics/elevation_mapping)：传感器和姿态不确定性参与地形估计；不因此把已有多层表示降成单层图。
- [Perceptive Locomotion through Nonlinear MPC](https://arxiv.org/abs/2208.08373)：足端区域、摆腿与躯干约束分别建模；本平台速度接口不等同论文底层接触控制。
- [Patchwork++](https://arxiv.org/abs/2207.11919)：局部地面分割可参考，但地面分类不证明植被下承重能力。
- [Traversability analysis with vision and terrain probing](https://arxiv.org/abs/2209.00334)：软地形存在视觉/几何歧义，可需要接触探测；不能把所有草当墙，也不能把不可见草下地面判定可走。

## 验证要求

分别覆盖平砖地无回波/固定自体/动态足端、缓起伏/密集起伏/坡阶、足间空隙与真实缺失接触、坑洞、细障碍/低顶/转向扫掠、多层、草地可见地面/仅冠层/遮挡硬物，以及升降摇晃、标定和采样时序偏差。

记录误拒绝、漏障碍、未知比例、拒绝原因、整段路线与CPU/内存/延迟口径。定点许可或纯几何单测不能替代完整导航和实机验证；失败候选与原始证据保留，达到准入后才发布。

## 动态观测来源核验

官方[Go2 LowState](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/idl/unitree_go/msg/dds_/_LowState_.py)有motor_state与uint32 tick，没有stamp/header。本次未取得Go2官方tick单位、设备wrap/reset或与LiDAR时钟映射依据，不能借用Go1的毫秒注释。

已有[SportModeState](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/idl/unitree_go/msg/dds_/_SportModeState_.py)提供stamp及foot_position_body/foot_speed_body，可沿现有高层只读订阅进一步验证足端观测，不必把全部关节FK作为唯一途径。但字段存在尚不证明末端参考点、足端顺序、估计滞后及逐点时间对齐，不立即作为删点依据，也不覆盖整条腿。

[官方电机索引](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/test/lowlevel/unitree_go2_const.py)足以确定FR/FL/RR/RL每组三轴hip/thigh/calf的索引；通用层只使用模型关节名。设备与URDF的实际零位/正方向仍需姿态资格证据，不自动猜符号修正。[只读订阅示例](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/test/lowlevel/read_lowstate.py)不需要创建lowcmd发布器，不能为读取状态复用会初始化控制的旧LowLevelController。

## 分辨率与处理结果来源（10月8日补充审计）

用户进一步询问：现场使用5cm体素，更细网格是否有意义、精度参数应否跟随点云间隔，以及后续是否仍使用原始完整点云。这里区分地图证据分辨率、运动采样间隔、传感器精度和机器人能力，不把它们混成一个5cm参数。

当前代码默认canonical cloud为5cm，MLS粗索引独立配置默认10cm；导航源尺度先从publication读取，有fine manifest时以其voxel_size_m为准。完整细观测存在时PCT使用实际源尺度，否则用粗索引。已有source≤coarse约束。显示点云因容量可变粗，不代表fine源地图也随之变粗。

1/2cm场景是通用配置反例测试，不是改现场体素。把5cm证据细分成1cm格不产生新观测；按源尺度半格检查机体扫掠，则是防止运动采样漏检，不提高地图测量精度。机器人尺寸、跨阶能力、定位误差及控制参数有独立物理来源，不能随体素同比缩放。当前路径/动作采样仍有固定上下限，尚未实现全面统一派生。

导航PCT与近场读取同一/nav/map3d处理结果；fine root/chunks按变化列更新，旧完整cloud兼容路径读取的也是处理后地图，不是原始雷达。原始扫描供LIO定位和采样时间处理；另有注册后dense显示分支保留原坐标/重复观测。语义投影当前经provider/device点云帧输入，没有走同一canonical自体过滤/5cm体素链。是否共享过滤、是否保留帧细节，应按时序和消费者目的审查，不能直接把当前相机投影帧替换成累计地图。

因此需统一实际分辨率来源和明确各分支用途，避免重复处理与误用；不因这次询问立即修改现场配置或重建用户地图。
