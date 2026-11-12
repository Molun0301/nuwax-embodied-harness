#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double resolution, body_length, body_width, body_height, body_to_ground;
    double max_velocity, max_acceleration, max_yaw_rate, planning_horizon, sample_dt;
    double body_center_x, body_center_y, body_z_min, body_z_max;
    double max_step_up, max_step_down, max_slope, max_yaw_acceleration;
    double max_reverse_velocity, max_lateral_velocity;
    double obstacle_clearance_m;
} ScanConfig;
typedef struct { double time, position[3], velocity[3], acceleration[3], yaw, yaw_rate; } ScanSample;
typedef struct { double stop[3], turn[3], route_point[3], route_arc; } ScanTerminalPoints;
typedef struct { double field_distance, body_clearance; int32_t occupied; } ScanPoseClearance;
/* 浅接触退出复用原检查的样本输出，不携带任何速度有效期。 */
typedef struct {
    double duration, reference_progress, reference_duration, local_target[3];
    double command_forward, command_lateral, command_yaw_rate, command_duration;
    ScanSample *samples; size_t sample_count;
    uint32_t replanned, execution_cycle_complete, execution_failed, motion_boundary_sources[3];
} ScanOutput;
typedef struct {
    double position[3], velocity[3], planning_velocity[3], yaw, yaw_rate, route_progress;
    int64_t timestamp_ns;
    uint32_t measured_velocity, measured_yaw_rate;
} ScanState;
/* 实测停稳口径由原生末端和运行时共同使用。 */
int nuwax_scan_motion_stopped(const ScanState *state);
/* 跟踪原因位；与 Rust track_reason 一一对应。 */
typedef enum {
    SCAN_TRACK_NO_TRAJECTORY = 1u << 0,
    SCAN_TRACK_DEVIATED = 1u << 1,
    SCAN_TRACK_FINISHED = 1u << 2,
    SCAN_TRACK_NO_CANDIDATE = 1u << 3,
    SCAN_TRACK_REPLAN_EARLY = 1u << 4,
    SCAN_TRACK_RECOVERY_FAILED = 1u << 5,
    SCAN_TRACK_REJECTED_TRAJECTORY = 1u << 6
} ScanTrackReason;
typedef struct {
    double command[3], phase, remaining, deviation;
    /* 1无轨迹，2偏离，4走完，8无合法候选，16提前请求，32恢复失败，64新曲线偏离（旧曲线继续）。 */
    uint32_t reason, limited, installed, turn_first;
} ScanTrackOutput;
void *nuwax_scan_map_create(const ScanConfig *config);
void nuwax_scan_map_destroy(void *map);
int nuwax_scan_map_surface(void *map,const double *obstacles,size_t count,const double *ground,size_t ground_count,
    const double *boundary,size_t boundary_count,const int32_t *forbidden,size_t forbidden_count);
int nuwax_scan_map_ingest(void *map,const double *points,size_t count,const uint32_t *ground,size_t ground_count,
    const double origin[3],const double body[3]);
/* 函数参数形式的静态占据初始化，数值夹具不向会话写测试开关。 */
void nuwax_scan_map_seed_occupied(void *map,const double *points,size_t count,const double body[3]);
void *nuwax_scan_map_snapshot(const void *map);
int nuwax_scan_snapshot_affects(const void *current,const void *previous,const void *trajectory,double from);
void nuwax_scan_snapshot_destroy(void *snapshot);
void nuwax_scan_terminal_points(const void *snapshot,const ScanConfig *config,const double goal[3],
    double yaw,double tolerance,const double *reference,size_t count,ScanTerminalPoints *output);
void nuwax_scan_pose_clearance(const void *snapshot,const double position[3],double yaw,ScanPoseClearance *output);
void *nuwax_scan_planner_create(void);
void nuwax_scan_planner_destroy(void *planner);
void *nuwax_scan_planner_plan(void *planner,const ScanConfig *config,const void *snapshot,
    const double *reference,size_t count,const ScanState *state,const void *active,double phase,
    char *error,size_t error_capacity);
void nuwax_scan_trajectory_destroy(void *trajectory);
void nuwax_scan_trajectory_samples(const void *trajectory,double from,double dt,ScanOutput *output);
void *nuwax_scan_tracker_create(void);
void nuwax_scan_tracker_destroy(void *tracker);
void nuwax_scan_tracker_clear(void *tracker);
void nuwax_scan_tracker_clear_trajectory(void *tracker);
void nuwax_scan_tracker_install(void *tracker,const void *trajectory);
void nuwax_scan_tracker_track(void *tracker,const ScanConfig *config,const void *snapshot,
    const ScanState *state,const double previous_command[3],double terminal_yaw,const double *terminal_position,ScanTrackOutput *output,ScanOutput *recovery,
    char *error,size_t error_capacity);
void nuwax_scan_free(ScanOutput *output);
#ifdef __cplusplus
}
#endif
