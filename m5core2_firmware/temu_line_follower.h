/*
 * ==============================================================================
 * Subsystem: Camera Vision Line-Following Engine for Mecanum Drive
 * Architecture: Powered by cl0ud (https://github.com/Ojas-sta/cl0ud-brand-engineering)
 * Co-Engineered by: Ojas Singh <ojasctc@gmail.com>
 * ==============================================================================
 * Pure C port of the TemuFollowe camera line-following algorithms.
 * Designed for embedded vision cameras (ESP32-CAM, K210, OpenMV, ARM Cortex).
 *
 * Core Features Preserved:
 *   - Multi-ROI line detection (Near, Lookahead, Wide search)
 *   - Color marker detection (Green = continue, Red = stop)
 *   - Runge-Kutta 4th Order (RK4) trajectory & heading prediction
 *   - Adaptive PID gain scheduling (error & derivative load boost)
 *   - Speed scaling (curves, stability, confidence, center deadzone bonus)
 *   - Intersection memory & dead-end backtracking (Temporal Buffer)
 *   - Mecanum 4x4 inverse kinematics conversion (vx, vy, yaw -> FL, FR, RL, RR)
 */

#ifndef TEMU_LINE_FOLLOWER_H
#define TEMU_LINE_FOLLOWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ==============================================================================
// CONFIGURATION CONSTANTS (Matching TemuFollowe config.py)
// ==============================================================================
#define TLF_KP                          1.10f
#define TLF_KI                          0.00f
#define TLF_KD                          0.18f
#define TLF_TURN_LIMIT                  0.80f
#define TLF_CENTER_DEADZONE             0.12f

#define TLF_ADAPTIVE_PID_ENABLED        true
#define TLF_ADAPTIVE_KP_ERROR_BOOST     0.55f
#define TLF_ADAPTIVE_KP_CONF_DROP       0.25f
#define TLF_ADAPTIVE_KD_ERROR_BOOST     0.35f
#define TLF_ADAPTIVE_KD_DERIV_BOOST     0.55f
#define TLF_ADAPTIVE_KI_ERROR_RED       0.70f
#define TLF_ADAPTIVE_DERIV_REF          4.00f
#define TLF_INTEGRAL_LIMIT              1.20f

#define TLF_LOW_CONF_SPEED_SCALE        0.65f
#define TLF_ERROR_SPEED_REDUCTION       0.55f
#define TLF_DERIV_SPEED_REDUCTION       0.35f
#define TLF_STRAIGHT_SPEED_BOOST        0.10f
#define TLF_CURVE_SPEED_REDUCTION       0.45f
#define TLF_CURVE_TURN_BOOST            0.35f
#define TLF_MIN_CURVE_SPEED_SCALE       0.45f

#define TLF_NEAR_TRAJ_WEIGHT            0.65f
#define TLF_LOOKAHEAD_TRAJ_WEIGHT       0.35f
#define TLF_LOOKAHEAD_CONF_MIN          0.25f
#define TLF_RK4_TRAJ_GAIN               10.0f
#define TLF_RK4_HEADING_GAIN            8.00f
#define TLF_RK4_MAX_DT                  0.05f

#define TLF_SHARP_TURN_SPEED            0.50f
#define TLF_MAX_TURN_SPEED              0.65f
#define TLF_STEER_INVERT                true
#define TLF_BASE_SPEED                  0.26f
#define TLF_MAX_SPEED                   0.42f
#define TLF_MIN_SPEED                   0.10f
#define TLF_SPIN_SEARCH_SPEED           0.32f
#define TLF_APPROACH_SPEED              0.24f
#define TLF_BACKTRACK_SPEED             0.18f
#define TLF_ROTATE_SPEED                0.38f
#define TLF_SLEW_RATE_PER_S             0.45f

#define TLF_LINE_LOST_TIMEOUT_S         4.00f
#define TLF_BACKTRACK_TIMEOUT_S         8.00f
#define TLF_MARKER_DELAY_S              1.00f
#define TLF_INTERSECTION_COOLDOWN_S     2.00f

#define TLF_ROI_Y_START_RATIO           0.67f
#define TLF_ROI_Y_END_RATIO             0.86f
#define TLF_ROI_X_START_RATIO           0.05f
#define TLF_ROI_X_END_RATIO             0.85f

#define TLF_LOOKAHEAD_Y_START_RATIO     0.42f
#define TLF_LOOKAHEAD_Y_END_RATIO       0.62f
#define TLF_LOOKAHEAD_X_START_RATIO     0.05f
#define TLF_LOOKAHEAD_X_END_RATIO       0.85f

#define TLF_WIDE_Y_START_RATIO          0.20f
#define TLF_WIDE_X_START_RATIO          0.00f
#define TLF_WIDE_X_END_RATIO            1.00f

#define TLF_BLACK_THRESHOLD             90
#define TLF_MIN_LINE_AREA               45
#define TLF_GREEN_DIFF_THRESHOLD        40
#define TLF_RED_DIFF_THRESHOLD          40
#define TLF_MIN_MARKER_AREA             40

#define TLF_MAX_WAYPOINTS               50
#define TLF_MAX_BRANCHES                4

// ==============================================================================
// DATA STRUCTURES
// ==============================================================================

typedef enum {
    TLF_STATE_FOLLOW = 0,
    TLF_STATE_APPROACH,
    TLF_STATE_SPIN_SEARCH,
    TLF_STATE_BACKTRACK,
    TLF_STATE_ROTATE,
    TLF_STATE_STOP,
    TLF_STATE_HALTED
} TLFState;

typedef struct {
    float vx;       // Planar Forward/Reverse velocity [-1.0, 1.0]
    float vy;       // Planar Strafe Right/Left velocity [-1.0, 1.0]
    float yaw;      // Planar Rotation rate [-1.0, 1.0] (+ turn right, - turn left)
    float w_fl;     // Front-Left Mecanum wheel [-1.0, 1.0]
    float w_fr;     // Front-Right Mecanum wheel [-1.0, 1.0]
    float w_rl;     // Rear-Left Mecanum wheel [-1.0, 1.0]
    float w_rr;     // Rear-Right Mecanum wheel [-1.0, 1.0]
    int8_t pwm_fl;  // Scaled motor duty cycle [-127, 127]
    int8_t pwm_fr;  // Scaled motor duty cycle [-127, 127]
    int8_t pwm_rl;  // Scaled motor duty cycle [-127, 127]
    int8_t pwm_rr;  // Scaled motor duty cycle [-127, 127]
} TLFMecanumVector;

typedef struct {
    float error;
    float heading;
} TLFTrajectoryState;

typedef struct {
    int cx;
    int cy;
    bool taken;
} TLFBranch;

typedef struct {
    float timestamp_s;
    TLFState state;
    float error;
    bool is_intersection;
    uint8_t branch_count;
    TLFBranch branches[TLF_MAX_BRANCHES];
    int8_t chosen_branch_idx;
    bool dead_end;
} TLFWaypoint;

typedef struct {
    TLFWaypoint waypoints[TLF_MAX_WAYPOINTS];
    uint16_t head;
    uint16_t count;
} TLFTemporalBuffer;

typedef struct {
    bool detected;
    float error;             // Normalized error [-1.0, 1.0] (0 = centered)
    float confidence;        // [0.0, 1.0]
    uint32_t area;
    int cx;
    int cy;
    bool green_marker;
    bool red_marker;
} TLFLineResult;

typedef struct {
    // State machine & timing
    TLFState state;
    float last_time_s;
    float applied_forward;
    float integral;
    float last_error;
    float search_direction;
    float spin_start_time_s;
    float backtrack_start_time_s;
    float rotate_settle_until_s;
    float last_intersection_time_s;
    int8_t backtrack_target_branch_idx;
    int backtrack_wp_index;
    bool dead_end_recorded;

    // Marker action
    bool halted;
    char pending_marker_color; // 'r', 'g', or 0
    float pending_marker_time_s;
    bool prev_red_marker;
    bool prev_green_marker;

    // Filters & Trajectory
    TLFTrajectoryState trajectory;
    TLFTemporalBuffer buffer;

    // Active PID gains
    float current_kp;
    float current_ki;
    float current_kd;
    float curve_sharpness;
    float curve_speed_scale;

    // Output vectors
    TLFMecanumVector vector;
} TLFController;

// ==============================================================================
// PUBLIC API
// ==============================================================================

/**
 * Initializes the line follower controller state.
 */
void tlf_init(TLFController* c, float current_time_s);

/**
 * Converts planar movement vectors (vx, vy, yaw) into 4-wheel Mecanum kinematics.
 */
void tlf_mecanum_convert(float vx, float vy, float yaw, TLFMecanumVector* out);

/**
 * Runge-Kutta 4th Order trajectory filter step.
 */
void tlf_rk4_step(TLFTrajectoryState* state, float target_error, float target_heading, 
                  float dt, float* pred_error, float* pred_heading);

/**
 * Adaptive PID gain scheduler based on error magnitude, derivative load, and confidence.
 */
void tlf_schedule_pid_gains(float base_kp, float base_ki, float base_kd, 
                            float error, float derivative, float line_conf,
                            float* out_kp, float* out_ki, float* out_kd);

/**
 * Processes a camera frame ROI for black line centroid and color markers.
 * Frame formats supported:
 *   format: 0 = Grayscale (1 byte/pixel)
 *           1 = RGB565 (2 bytes/pixel, Little-Endian)
 *           2 = RGB888 / BGR888 (3 bytes/pixel)
 */
TLFLineResult tlf_detect_line(const uint8_t* frame, int width, int height, int format,
                              float y_start_ratio, float y_end_ratio,
                              float x_start_ratio, float x_end_ratio);

/**
 * Main control step: runs vision analysis, trajectory prediction, adaptive PID,
 * state machine, and computes the Mecanum movement vector.
 *
 * Parameters:
 *   c: Pointer to initialized TLFController
 *   frame: Camera frame buffer
 *   width, height: Frame dimensions
 *   format: Frame pixel format (0=Gray, 1=RGB565, 2=RGB888)
 *   now_s: Current timestamp in seconds
 *
 * Returns:
 *   Pointer to controller's updated TLFMecanumVector.
 */
const TLFMecanumVector* tlf_update(TLFController* c, const uint8_t* frame, 
                                   int width, int height, int format, float now_s);

/**
 * Formats a serial UART packet string for streaming to Main ESP32 / M5Core2.
 * Output format: "BINZO:MEC,<vx*100>,<vy*100>,<yaw*100>,<pwm_fl>,<pwm_fr>,<pwm_rl>,<pwm_rr>,<state_id>\n"
 */
int tlf_format_uart_packet(const TLFController* c, char* out_buf, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif // TEMU_LINE_FOLLOWER_H
