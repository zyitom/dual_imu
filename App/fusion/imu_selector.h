#ifndef IMU_SELECTOR_H
#define IMU_SELECTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_SELECTOR_LANE_COUNT (2U)
#define IMU_SELECTOR_LANE_MASK(lane) ((uint8_t)(1U << (uint8_t)(lane)))

typedef enum
{
    IMU_SELECTOR_LANE_0 = 0,
    IMU_SELECTOR_LANE_1 = 1,
    IMU_SELECTOR_LANE_NONE = 0xFF
} imu_selector_lane_t;

typedef enum
{
    IMU_SELECTOR_HEALTHY = 0,
    IMU_SELECTOR_SUSPECT,
    IMU_SELECTOR_FAULT,
    IMU_SELECTOR_AMBIGUOUS
} imu_selector_state_t;

typedef enum
{
    IMU_SELECTOR_HINT_NONE = 0,
    IMU_SELECTOR_HINT_LANE_0_BAD,
    IMU_SELECTOR_HINT_LANE_1_BAD
} imu_selector_hint_t;

typedef enum
{
    IMU_SELECTOR_REASON_NONE = 0U,
    IMU_SELECTOR_REASON_LANE_0_HARD = (1U << 0),
    IMU_SELECTOR_REASON_LANE_1_HARD = (1U << 1),
    IMU_SELECTOR_REASON_LANE_0_INPUT = (1U << 2),
    IMU_SELECTOR_REASON_LANE_1_INPUT = (1U << 3),
    IMU_SELECTOR_REASON_NIS_INVALID = (1U << 4),
    IMU_SELECTOR_REASON_NIS_HIGH = (1U << 5),
    IMU_SELECTOR_REASON_EXTERNAL_HINT = (1U << 6),
    IMU_SELECTOR_REASON_NO_OUTPUT = (1U << 7)
} imu_selector_reason_t;

typedef struct
{
    /*
     * Both lanes must cover the same time window and use one common body frame.
     * Covariance should include timing/alignment uncertainty; the NIS calculation
     * assumes the remaining lane errors are independent.
     */
    float delta_angle_rad[3];
    float covariance_rad2[3][3];
    bool window_valid;

    /*
     * Any nonzero bit is treated as locally attributable, definitive evidence.
     * Cross-lane residuals must never be fed back as hard fault flags.
     */
    uint32_t hard_fault_flags;
} imu_selector_lane_input_t;

typedef struct
{
    imu_selector_lane_input_t lane[IMU_SELECTOR_LANE_COUNT];

    /* False pauses residual counters, but does not suppress local hard faults. */
    bool residual_enabled;

    /*
     * Optional independent evidence, for example a time-aligned external aid.
     * It only attributes a high cross-lane residual; it is never used alone.
     */
    imu_selector_hint_t isolation_hint;
} imu_selector_input_t;

typedef struct
{
    float nis_enter_threshold;
    float nis_clear_threshold;
    float covariance_floor_rad2;
    uint16_t suspect_enter_windows;
    uint16_t ambiguous_enter_windows;
    uint16_t soft_fault_confirm_windows;
    uint16_t clear_windows;
    uint16_t isolated_recovery_windows;
} imu_selector_config_t;

typedef struct
{
    imu_selector_state_t state;
    imu_selector_lane_t selected_lane;
    float residual_rad[3];
    float residual_nis;
    bool residual_valid;
    bool selection_changed;
    uint8_t hard_fault_mask;
    uint8_t isolated_mask;
    uint8_t usable_mask;
    uint8_t suspect_mask;
    uint32_t reason_flags;
    uint16_t mismatch_streak;
    uint16_t clear_streak;
} imu_selector_result_t;

typedef struct
{
    imu_selector_config_t config;
    imu_selector_state_t state;
    imu_selector_lane_t preferred_lane;
    imu_selector_lane_t selected_lane;
    uint8_t hard_latched_mask;
    uint8_t soft_latched_mask;
    uint16_t mismatch_streak;
    uint16_t clear_streak;
    uint16_t hint_streak[IMU_SELECTOR_LANE_COUNT];
    uint16_t recovery_streak[IMU_SELECTOR_LANE_COUNT];
    bool initialized;
} imu_selector_t;

void imu_selector_default_config(imu_selector_config_t *config);

bool imu_selector_init(imu_selector_t *selector,
                       const imu_selector_config_t *config,
                       imu_selector_lane_t preferred_lane);

bool imu_selector_update(imu_selector_t *selector,
                         const imu_selector_input_t *input,
                         imu_selector_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
