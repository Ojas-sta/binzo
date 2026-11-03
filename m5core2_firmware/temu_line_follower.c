/*
 * ==============================================================================
 * Subsystem: Camera Vision Line-Following Engine for Mecanum Drive
 * Architecture: Powered by cl0ud (https://github.com/Ojas-sta/cl0ud-brand-engineering)
 * Co-Engineered by: Ojas Singh <ojasctc@gmail.com>
 * ==============================================================================
 */

#include "temu_line_follower.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// ==============================================================================
// INTERNAL UTILITIES
// ==============================================================================

static inline float tlf_clamp(float val, float low, float high) {
    if (val < low) return low;
    if (val > high) return high;
    return val;
}

static inline float tlf_slew(float current, float target, float max_step) {
    if (target > current) {
        float next = current + max_step;
        return (next < target) ? next : target;
    }
    if (target < current) {
        float next = current - max_step;
        return (next > target) ? next : target;
    }
    return current;
}

// ==============================================================================
// MECANUM 4X4 KINEMATICS CONVERSION
// ==============================================================================

void tlf_mecanum_convert(float vx, float vy, float yaw, TLFMecanumVector* out) {
    out->vx = tlf_clamp(vx, -1.0f, 1.0f);
    out->vy = tlf_clamp(vy, -1.0f, 1.0f);
    out->yaw = tlf_clamp(yaw, -1.0f, 1.0f);

    // Standard Mecanum Inverse Kinematics
    float w_fl = out->vx + out->vy + out->yaw;
    float w_fr = out->vx - out->vy - out->yaw;
    float w_rl = out->vx - out->vy + out->yaw;
    float w_rr = out->vx + out->vy - out->yaw;

    // Normalize so wheel magnitudes never exceed 1.0, preserving directional vector
    float max_mag = 1.0f;
    float a_fl = fabsf(w_fl);
    float a_fr = fabsf(w_fr);
    float a_rl = fabsf(w_rl);
    float a_rr = fabsf(w_rr);

    if (a_fl > max_mag) max_mag = a_fl;
    if (a_fr > max_mag) max_mag = a_fr;
    if (a_rl > max_mag) max_mag = a_rl;
    if (a_rr > max_mag) max_mag = a_rr;

    w_fl /= max_mag;
    w_fr /= max_mag;
    w_rl /= max_mag;
    w_rr /= max_mag;

    out->w_fl = w_fl;
    out->w_fr = w_fr;
    out->w_rl = w_rl;
    out->w_rr = w_rr;

    // Convert to standard signed 8-bit motor driver duty (-127 to 127)
    out->pwm_fl = (int8_t)roundf(w_fl * 127.0f);
    out->pwm_fr = (int8_t)roundf(w_fr * 127.0f);
    out->pwm_rl = (int8_t)roundf(w_rl * 127.0f);
    out->pwm_rr = (int8_t)roundf(w_rr * 127.0f);
}

// ==============================================================================
// RUNGE-KUTTA 4TH ORDER (RK4) TRAJECTORY PREDICTOR
// ==============================================================================

static inline void rk4_derivs(float error, float heading, float target_err, float target_hdg,
                              float* d_error, float* d_heading) {
    *d_error   = TLF_RK4_TRAJ_GAIN * (target_err - error);
    *d_heading = TLF_RK4_HEADING_GAIN * (target_hdg - heading);
}

void tlf_rk4_step(TLFTrajectoryState* state, float target_error, float target_heading, 
                  float dt, float* pred_error, float* pred_heading) {
    dt = tlf_clamp(dt, 0.001f, TLF_RK4_MAX_DT);

    float k1_err, k1_hdg;
    rk4_derivs(state->error, state->heading, target_error, target_heading, &k1_err, &k1_hdg);

    float k2_err, k2_hdg;
    rk4_derivs(state->error + k1_err * dt * 0.5f, state->heading + k1_hdg * dt * 0.5f, 
               target_error, target_heading, &k2_err, &k2_hdg);

    float k3_err, k3_hdg;
    rk4_derivs(state->error + k2_err * dt * 0.5f, state->heading + k2_hdg * dt * 0.5f, 
               target_error, target_heading, &k3_err, &k3_hdg);

    float k4_err, k4_hdg;
    rk4_derivs(state->error + k3_err * dt, state->heading + k3_hdg * dt, 
               target_error, target_heading, &k4_err, &k4_hdg);

    state->error   += (dt / 6.0f) * (k1_err + 2.0f * k2_err + 2.0f * k3_err + k4_err);
    state->heading += (dt / 6.0f) * (k1_hdg + 2.0f * k2_hdg + 2.0f * k3_hdg + k4_hdg);

    state->error   = tlf_clamp(state->error, -1.0f, 1.0f);
    state->heading = tlf_clamp(state->heading, -1.0f, 1.0f);

    if (pred_error)   *pred_error   = state->error;
    if (pred_heading) *pred_heading = state->heading;
}

// ==============================================================================
// ADAPTIVE PID GAIN SCHEDULER
// ==============================================================================

void tlf_schedule_pid_gains(float base_kp, float base_ki, float base_kd, 
                            float error, float derivative, float line_conf,
                            float* out_kp, float* out_ki, float* out_kd) {
    if (!TLF_ADAPTIVE_PID_ENABLED) {
        *out_kp = base_kp;
        *out_ki = base_ki;
        *out_kd = base_kd;
        return;
    }

    float abs_err = fabsf(error);
    float deriv_load = tlf_clamp(fabsf(derivative) / TLF_ADAPTIVE_DERIV_REF, 0.0f, 1.0f);
    float conf = tlf_clamp(line_conf, 0.0f, 1.0f);
    float conf_loss = 1.0f - conf;

    float kp = base_kp * (1.0f + TLF_ADAPTIVE_KP_ERROR_BOOST * abs_err - TLF_ADAPTIVE_KP_CONF_DROP * conf_loss);
    float ki_factor = 1.0f - TLF_ADAPTIVE_KI_ERROR_RED * abs_err;
    if (ki_factor < 0.0f) ki_factor = 0.0f;
    float ki = base_ki * ki_factor;

    float kd = base_kd * (1.0f + TLF_ADAPTIVE_KD_ERROR_BOOST * abs_err + TLF_ADAPTIVE_KD_DERIV_BOOST * deriv_load);

    *out_kp = (kp > 0.0f) ? kp : 0.0f;
    *out_ki = (ki > 0.0f) ? ki : 0.0f;
    *out_kd = (kd > 0.0f) ? kd : 0.0f;
}

// ==============================================================================
// CAMERA VISION: ROI CENTROID & MARKER DETECTION
// ==============================================================================

TLFLineResult tlf_detect_line(const uint8_t* frame, int width, int height, int format,
                              float y_start_ratio, float y_end_ratio,
                              float x_start_ratio, float x_end_ratio) {
    TLFLineResult res;
    memset(&res, 0, sizeof(res));

    if (!frame || width <= 0 || height <= 0) return res;

    int y0 = (int)(height * y_start_ratio);
    int y1 = (int)(height * y_end_ratio);
    int x0 = (int)(width * x_start_ratio);
    int x1 = (int)(width * x_end_ratio);

    if (y0 < 0) y0 = 0;
    if (y1 > height) y1 = height;
    if (x0 < 0) x0 = 0;
    if (x1 > width) x1 = width;

    int roi_w = x1 - x0;
    if (roi_w <= 0 || y1 <= y0) return res;

    uint32_t m00 = 0;
    uint32_t m10 = 0;
    uint32_t m01 = 0;
    uint32_t green_pixels = 0;
    uint32_t red_pixels = 0;

    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            uint8_t r = 0, g = 0, b = 0, gray = 0;

            if (format == 0) {
                // Grayscale
                gray = frame[y * width + x];
            } else if (format == 1) {
                // RGB565 (Little Endian)
                uint16_t rgb565 = ((const uint16_t*)frame)[y * width + x];
                r = (uint8_t)(((rgb565 >> 11) & 0x1F) << 3);
                g = (uint8_t)(((rgb565 >> 5) & 0x3F) << 2);
                b = (uint8_t)((rgb565 & 0x1F) << 3);
                gray = (uint8_t)((r * 299 + g * 587 + b * 114) / 1000);
            } else {
                // RGB888 / BGR888
                const uint8_t* p = &frame[(y * width + x) * 3];
                b = p[0];
                g = p[1];
                r = p[2];
                gray = (uint8_t)((r * 299 + g * 587 + b * 114) / 1000);
            }

            // Color marker checks (G-R > thresh and G-B > thresh)
            if (format != 0) {
                if ((int)g - (int)r >= TLF_GREEN_DIFF_THRESHOLD && (int)g - (int)b >= TLF_GREEN_DIFF_THRESHOLD) {
                    green_pixels++;
                    continue; // exclude marker color from black line mask
                }
                if ((int)r - (int)g >= TLF_RED_DIFF_THRESHOLD && (int)r - (int)b >= TLF_RED_DIFF_THRESHOLD) {
                    red_pixels++;
                    continue; // exclude marker color from black line mask
                }
            }

            // Black line threshold check (< 90)
            if (gray < TLF_BLACK_THRESHOLD) {
                m00++;
                m10 += (uint32_t)x;
                m01 += (uint32_t)y;
            }
        }
    }

    res.green_marker = (green_pixels >= TLF_MIN_MARKER_AREA);
    res.red_marker   = (red_pixels >= TLF_MIN_MARKER_AREA);
    res.area = m00;

    if (m00 >= TLF_MIN_LINE_AREA) {
        res.detected = true;
        res.cx = (int)(m10 / m00);
        res.cy = (int)(m01 / m00);

        int center_x_global = x0 + roi_w / 2;
        float half_roi_w = (float)roi_w / 2.0f;
        res.error = tlf_clamp((float)(res.cx - center_x_global) / (half_roi_w > 1.0f ? half_roi_w : 1.0f), -1.0f, 1.0f);
        res.confidence = tlf_clamp((float)m00 / (float)(TLF_MIN_LINE_AREA * 8), 0.0f, 1.0f);
    } else {
        res.detected = false;
        res.error = 0.0f;
        res.confidence = 0.0f;
    }

    return res;
}

// ==============================================================================
// CONTROLLER INITIALIZATION
// ==============================================================================

void tlf_init(TLFController* c, float current_time_s) {
    memset(c, 0, sizeof(TLFController));
    c->state = TLF_STATE_FOLLOW;
    c->last_time_s = current_time_s;
    c->search_direction = 1.0f;
    c->current_kp = TLF_KP;
    c->current_ki = TLF_KI;
    c->current_kd = TLF_KD;
    c->curve_speed_scale = 1.0f;
    c->backtrack_target_branch_idx = -1;
    c->backtrack_wp_index = -1;
}

// ==============================================================================
// MAIN CONTROL STEP
// ==============================================================================

const TLFMecanumVector* tlf_update(TLFController* c, const uint8_t* frame, 
                                   int width, int height, int format, float now_s) {
    float dt = now_s - c->last_time_s;
    if (dt < 0.001f) dt = 0.001f;
    c->last_time_s = now_s;
    float max_step = TLF_SLEW_RATE_PER_S * dt;

    if (c->halted) {
        tlf_mecanum_convert(0.0f, 0.0f, 0.0f, &c->vector);
        return &c->vector;
    }

    // 1. Process Near ROI (Main Steering Reference)
    TLFLineResult normal = tlf_detect_line(frame, width, height, format,
                                          TLF_ROI_Y_START_RATIO, TLF_ROI_Y_END_RATIO,
                                          TLF_ROI_X_START_RATIO, TLF_ROI_X_END_RATIO);

    float target_forward = 0.0f;
    float target_turn = 0.0f;

    // 2. State Machine Execution
    if (c->state == TLF_STATE_BACKTRACK) {
        // Reverse along the track to backtrack to last intersection
        if (normal.detected) {
            float raw_turn = tlf_clamp(TLF_KP * normal.error, -TLF_TURN_LIMIT, TLF_TURN_LIMIT);
            float motor_turn = TLF_STEER_INVERT ? -raw_turn : raw_turn;
            target_forward = -TLF_BACKTRACK_SPEED;
            target_turn = motor_turn;
        } else {
            // Wide search while reversing
            TLFLineResult wide = tlf_detect_line(frame, width, height, format,
                                                TLF_WIDE_Y_START_RATIO, 1.0f,
                                                TLF_WIDE_X_START_RATIO, TLF_WIDE_X_END_RATIO);
            if (wide.detected) {
                float raw_turn = tlf_clamp(TLF_KP * wide.error, -TLF_TURN_LIMIT, TLF_TURN_LIMIT);
                target_forward = -TLF_APPROACH_SPEED;
                target_turn = TLF_STEER_INVERT ? -raw_turn : raw_turn;
            } else {
                if (c->backtrack_start_time_s > 0 && (now_s - c->backtrack_start_time_s > TLF_BACKTRACK_TIMEOUT_S)) {
                    c->state = TLF_STATE_STOP;
                } else {
                    target_forward = 0.0f;
                    target_turn = TLF_SPIN_SEARCH_SPEED * c->search_direction;
                }
            }
        }
    } else if (c->state == TLF_STATE_ROTATE) {
        if (c->rotate_settle_until_s > 0 && now_s < c->rotate_settle_until_s) {
            target_forward = 0.0f;
            target_turn = 0.0f;
        } else {
            c->rotate_settle_until_s = 0.0f;
            if (now_s - c->backtrack_start_time_s > TLF_BACKTRACK_TIMEOUT_S) {
                c->state = TLF_STATE_STOP;
            } else if (normal.detected && fabsf(normal.error) < TLF_CENTER_DEADZONE * 0.5f) {
                c->state = TLF_STATE_FOLLOW;
                c->integral = 0.0f;
                c->last_error = 0.0f;
            } else {
                target_forward = 0.0f;
                target_turn = TLF_ROTATE_SPEED * (TLF_STEER_INVERT ? -1.0f : 1.0f);
            }
        }
    } else if (normal.detected) {
        // ----- NORMAL LINE FOLLOWING -----
        c->state = TLF_STATE_FOLLOW;
        c->dead_end_recorded = false;

        // Lookahead ROI (Anticipate Upcoming Curves)
        TLFLineResult lookahead = tlf_detect_line(frame, width, height, format,
                                                  TLF_LOOKAHEAD_Y_START_RATIO, TLF_LOOKAHEAD_Y_END_RATIO,
                                                  TLF_LOOKAHEAD_X_START_RATIO, TLF_LOOKAHEAD_X_END_RATIO);

        float curve = 0.0f;
        float target_traj_err = normal.error;

        if (lookahead.detected && lookahead.confidence >= TLF_LOOKAHEAD_CONF_MIN) {
            curve = lookahead.error - normal.error;
            float speed_factor = tlf_clamp(fabsf(c->applied_forward) / TLF_BASE_SPEED, 0.0f, 1.0f);
            float off_center_factor = tlf_clamp(fabsf(normal.error) / TLF_CENTER_DEADZONE, 0.0f, 1.0f);
            float lookahead_trust = speed_factor * (1.0f - 0.5f * off_center_factor);
            float lookahead_weight = TLF_LOOKAHEAD_TRAJ_WEIGHT * lookahead_trust;
            float near_weight = TLF_NEAR_TRAJ_WEIGHT + TLF_LOOKAHEAD_TRAJ_WEIGHT * (1.0f - lookahead_trust);
            target_traj_err = (near_weight * normal.error) + (lookahead_weight * lookahead.error);
        }

        // RK4 Trajectory Predictor
        float pred_error = 0.0f, pred_heading = 0.0f;
        tlf_rk4_step(&c->trajectory, target_traj_err, curve, dt, &pred_error, &pred_heading);

        c->curve_sharpness = tlf_clamp(fabsf(pred_heading), 0.0f, 1.0f);
        c->curve_speed_scale = tlf_clamp(1.0f - TLF_CURVE_SPEED_REDUCTION * c->curve_sharpness, 
                                         TLF_MIN_CURVE_SPEED_SCALE, 1.0f);

        float error = pred_error;
        c->integral += error * dt;
        c->integral = tlf_clamp(c->integral, -TLF_INTEGRAL_LIMIT, TLF_INTEGRAL_LIMIT);
        float derivative = (error - c->last_error) / dt;
        c->last_error = error;

        // Adaptive PID Gains
        tlf_schedule_pid_gains(TLF_KP, TLF_KI, TLF_KD, error, derivative, normal.confidence,
                               &c->current_kp, &c->current_ki, &c->current_kd);

        float raw_turn = (c->current_kp * error) + (c->current_ki * c->integral) + (c->current_kd * derivative);
        raw_turn *= (1.0f + TLF_CURVE_TURN_BOOST * c->curve_sharpness);
        raw_turn = tlf_clamp(raw_turn, -TLF_TURN_LIMIT, TLF_TURN_LIMIT);
        float motor_turn = TLF_STEER_INVERT ? -raw_turn : raw_turn;

        float abs_error = fabsf(error);
        if (abs_error <= TLF_CENTER_DEADZONE) {
            float straight_bonus = TLF_STRAIGHT_SPEED_BOOST * (1.0f - abs_error / TLF_CENTER_DEADZONE);
            target_forward = TLF_BASE_SPEED + straight_bonus;
            target_turn = motor_turn;
        } else {
            float blend = tlf_clamp((abs_error - TLF_CENTER_DEADZONE) / (1.0f - TLF_CENTER_DEADZONE), 0.0f, 1.0f);
            float turn_sign = (motor_turn >= 0.0f) ? 1.0f : -1.0f;
            target_forward = TLF_BASE_SPEED * (1.0f - blend);
            target_turn = motor_turn * (1.0f - blend) + (TLF_SHARP_TURN_SPEED * turn_sign) * blend;
        }

        // Stability & Confidence Speed Scaling
        float deriv_load = tlf_clamp(fabsf(derivative) / TLF_ADAPTIVE_DERIV_REF, 0.0f, 1.0f);
        float stability = 1.0f - (TLF_ERROR_SPEED_REDUCTION * abs_error) - (TLF_DERIV_SPEED_REDUCTION * deriv_load);
        target_forward *= tlf_clamp(stability, TLF_MIN_SPEED / TLF_BASE_SPEED, 1.0f);
        target_forward *= c->curve_speed_scale;

        if (normal.confidence < 0.5f) {
            float conf_scale = TLF_LOW_CONF_SPEED_SCALE + ((1.0f - TLF_LOW_CONF_SPEED_SCALE) * normal.confidence * 2.0f);
            target_forward *= tlf_clamp(conf_scale, TLF_LOW_CONF_SPEED_SCALE, 1.0f);
        }

        target_forward = tlf_clamp(target_forward, TLF_MIN_SPEED, TLF_MAX_SPEED);
        c->search_direction = (error >= 0.0f) ? (TLF_STEER_INVERT ? -1.0f : 1.0f) : (TLF_STEER_INVERT ? 1.0f : -1.0f);
        c->spin_start_time_s = 0.0f;

        // Color Marker Logic
        if (normal.red_marker && !c->prev_red_marker) {
            c->pending_marker_color = 'r';
            c->pending_marker_time_s = now_s + TLF_MARKER_DELAY_S;
        } else if (normal.green_marker && !c->prev_green_marker && c->pending_marker_color == 0) {
            c->pending_marker_color = 'g';
            c->pending_marker_time_s = now_s + TLF_MARKER_DELAY_S;
        }
        c->prev_green_marker = normal.green_marker;
        c->prev_red_marker = normal.red_marker;

    } else {
        // ----- LINE LOST: WIDE APPROACH OR SPIN SEARCH -----
        c->prev_green_marker = false;
        c->prev_red_marker = false;

        TLFLineResult wide = tlf_detect_line(frame, width, height, format,
                                            TLF_WIDE_Y_START_RATIO, 1.0f,
                                            TLF_WIDE_X_START_RATIO, TLF_WIDE_X_END_RATIO);

        if (wide.detected) {
            c->state = TLF_STATE_APPROACH;
            c->dead_end_recorded = false;
            float raw_turn = tlf_clamp(TLF_KP * wide.error, -TLF_TURN_LIMIT, TLF_TURN_LIMIT);
            target_forward = TLF_APPROACH_SPEED;
            target_turn = TLF_STEER_INVERT ? -raw_turn : raw_turn;
            c->search_direction = (wide.error >= 0.0f) ? (TLF_STEER_INVERT ? -1.0f : 1.0f) : (TLF_STEER_INVERT ? 1.0f : -1.0f);
            c->spin_start_time_s = 0.0f;
        } else {
            c->state = TLF_STATE_SPIN_SEARCH;
            if (c->spin_start_time_s <= 0.0f) {
                c->spin_start_time_s = now_s;
            }

            if (now_s - c->spin_start_time_s < TLF_LINE_LOST_TIMEOUT_S) {
                target_forward = 0.0f;
                target_turn = -TLF_SPIN_SEARCH_SPEED * c->search_direction;
            } else {
                // Timeout searching: dead end reached -> switch to backtrack
                c->state = TLF_STATE_BACKTRACK;
                c->backtrack_start_time_s = now_s;
                target_forward = 0.0f;
                target_turn = 0.0f;
            }
        }
    }

    // 3. Delayed Marker Action
    if (c->pending_marker_color != 0 && now_s >= c->pending_marker_time_s) {
        if (c->pending_marker_color == 'r') {
            c->halted = true;
            c->state = TLF_STATE_HALTED;
            target_forward = 0.0f;
            target_turn = 0.0f;
        }
        c->pending_marker_color = 0;
        c->pending_marker_time_s = 0.0f;
    }

    // 4. Rate-Limiting & Mecanum Kinematics Output Conversion
    if (c->state == TLF_STATE_STOP || c->state == TLF_STATE_HALTED) {
        c->applied_forward = 0.0f;
        tlf_mecanum_convert(0.0f, 0.0f, 0.0f, &c->vector);
    } else if (c->state == TLF_STATE_ROTATE) {
        c->applied_forward = 0.0f;
        float turn_comp = tlf_clamp(target_turn, -TLF_ROTATE_SPEED, TLF_ROTATE_SPEED);
        tlf_mecanum_convert(0.0f, 0.0f, turn_comp, &c->vector);
    } else {
        c->applied_forward = tlf_slew(c->applied_forward, target_forward, max_step);
        float turn_comp = tlf_clamp(target_turn, -TLF_MAX_TURN_SPEED, TLF_MAX_TURN_SPEED);
        // Movement vectors: vx = forward, vy = 0.0 (strafe), yaw = turn_comp
        tlf_mecanum_convert(c->applied_forward, 0.0f, turn_comp, &c->vector);
    }

    return &c->vector;
}

// ==============================================================================
// UART SERIALIZATION (SEND TO M5CORE2)
// ==============================================================================

int tlf_format_uart_packet(const TLFController* c, char* out_buf, size_t max_len) {
    if (!c || !out_buf || max_len < 40) return 0;

    // Packet format: BINZO:MEC,<vx*100>,<vy*100>,<yaw*100>,<pwm_fl>,<pwm_fr>,<pwm_rl>,<pwm_rr>,<state>\n
    return snprintf(out_buf, max_len, 
                    "BINZO:MEC,%d,%d,%d,%d,%d,%d,%d,%d\n",
                    (int)roundf(c->vector.vx * 100.0f),
                    (int)roundf(c->vector.vy * 100.0f),
                    (int)roundf(c->vector.yaw * 100.0f),
                    c->vector.pwm_fl,
                    c->vector.pwm_fr,
                    c->vector.pwm_rl,
                    c->vector.pwm_rr,
                    (int)c->state);
}
