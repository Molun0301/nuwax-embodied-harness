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
} ScanConfig;

typedef struct {
    double time, position[3], velocity[3], acceleration[3], yaw, yaw_rate;
} ScanSample;

typedef struct {
    double duration, reference_progress, reference_duration, local_target[3];
    double command_forward, command_yaw_rate, command_duration;
    ScanSample *samples;
    size_t sample_count;
    uint32_t replanned;
} ScanOutput;

int nuwax_scan_plan(const ScanConfig *config, const double *obstacles, size_t obstacle_count,
    const double *ground, size_t ground_count, const double *reference, size_t reference_count,
    const double start_position[3], const double start_velocity[3], const double start_acceleration[3],
    double start_yaw, double start_yaw_rate, const double *boundary, size_t boundary_count, const int32_t *forbidden_columns, size_t forbidden_count, double goal_yaw, int64_t odometry_timestamp_ns, ScanOutput *output, char *error, size_t error_capacity);
void *nuwax_scan_session_create(void);
void nuwax_scan_session_destroy(void *session);
void nuwax_scan_session_reset(void *session);
/* 实验台对照：enabled为0时跳过局部A*与避障优化，直接跟随全局参考。默认开启。 */
void nuwax_scan_session_set_local_planning(void *session, int enabled);
uint64_t nuwax_scan_session_plan_count(const void *session);
int nuwax_scan_session_plan(void *session,const ScanConfig *config,const double *obstacles,size_t obstacle_count,
    const double *ground,size_t ground_count,const double *reference,size_t reference_count,
    const double start_position[3],const double start_velocity[3],const double start_acceleration[3],
    double start_yaw,double start_yaw_rate,const double *boundary,size_t boundary_count,const int32_t *forbidden_columns,
    size_t forbidden_count,double goal_yaw,int64_t odometry_timestamp_ns,ScanOutput *output,char *error,size_t error_capacity);
void nuwax_scan_free(ScanOutput *output);
#ifdef __cplusplus
}
#endif
