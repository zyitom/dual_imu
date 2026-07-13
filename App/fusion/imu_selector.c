#include "imu_selector.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_SELECTOR_CHI_SQUARE_3DOF_95 (7.814728f)
#define IMU_SELECTOR_CHI_SQUARE_3DOF_99 (11.344867f)

static uint16_t increment_saturated(uint16_t value)
{
    return (value < UINT16_MAX) ? (uint16_t)(value + 1U) : UINT16_MAX;
}

static bool config_is_valid(const imu_selector_config_t *config)
{
    return (config != NULL) && isfinite(config->nis_enter_threshold) &&
           isfinite(config->nis_clear_threshold) &&
           isfinite(config->covariance_floor_rad2) &&
           (config->nis_enter_threshold > config->nis_clear_threshold) &&
           (config->nis_clear_threshold > 0.0f) &&
           (config->covariance_floor_rad2 > 0.0f) &&
           (config->suspect_enter_windows > 0U) &&
           (config->ambiguous_enter_windows >= config->suspect_enter_windows) &&
           (config->soft_fault_confirm_windows >= config->suspect_enter_windows) &&
           (config->clear_windows > 0U) &&
           (config->isolated_recovery_windows > 0U);
}

static bool covariance_is_valid(const float covariance[3][3], float floor_rad2)
{
    float matrix[3][3];

    for (uint32_t row = 0U; row < 3U; ++row) {
        for (uint32_t column = 0U; column < 3U; ++column) {
            if (!isfinite(covariance[row][column]))
                return false;
            matrix[row][column] = 0.5f *
                                  (covariance[row][column] + covariance[column][row]);
        }
        matrix[row][row] += floor_rad2;
    }

    const float l00_sq = matrix[0][0];
    if (!(l00_sq > 0.0f))
        return false;
    const float l00 = sqrtf(l00_sq);
    const float l10 = matrix[1][0] / l00;
    const float l20 = matrix[2][0] / l00;
    const float l11_sq = matrix[1][1] - (l10 * l10);
    if (!(l11_sq > 0.0f))
        return false;
    const float l11 = sqrtf(l11_sq);
    const float l21 = (matrix[2][1] - (l20 * l10)) / l11;
    const float l22_sq = matrix[2][2] - (l20 * l20) - (l21 * l21);

    return isfinite(l22_sq) && (l22_sq > 0.0f);
}

static bool lane_numeric_is_valid(const imu_selector_lane_input_t *lane,
                                  float floor_rad2)
{
    if (!lane->window_valid)
        return false;

    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        if (!isfinite(lane->delta_angle_rad[axis]))
            return false;
    }

    return covariance_is_valid(lane->covariance_rad2, floor_rad2);
}

static bool calculate_nis(const imu_selector_input_t *input,
                          float covariance_floor_rad2,
                          float residual[3],
                          float *nis)
{
    float covariance[3][3];

    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        residual[axis] = input->lane[0].delta_angle_rad[axis] -
                         input->lane[1].delta_angle_rad[axis];
    }

    for (uint32_t row = 0U; row < 3U; ++row) {
        for (uint32_t column = 0U; column < 3U; ++column) {
            const float sum = input->lane[0].covariance_rad2[row][column] +
                              input->lane[1].covariance_rad2[row][column];
            const float transpose_sum =
                input->lane[0].covariance_rad2[column][row] +
                input->lane[1].covariance_rad2[column][row];
            covariance[row][column] = 0.5f * (sum + transpose_sum);
        }
        covariance[row][row] += covariance_floor_rad2;
    }

    const float l00_sq = covariance[0][0];
    if (!(l00_sq > 0.0f))
        return false;
    const float l00 = sqrtf(l00_sq);
    const float l10 = covariance[1][0] / l00;
    const float l20 = covariance[2][0] / l00;
    const float l11_sq = covariance[1][1] - (l10 * l10);
    if (!(l11_sq > 0.0f))
        return false;
    const float l11 = sqrtf(l11_sq);
    const float l21 = (covariance[2][1] - (l20 * l10)) / l11;
    const float l22_sq = covariance[2][2] - (l20 * l20) - (l21 * l21);
    if (!(l22_sq > 0.0f))
        return false;
    const float l22 = sqrtf(l22_sq);

    const float y0 = residual[0] / l00;
    const float y1 = (residual[1] - (l10 * y0)) / l11;
    const float y2 = (residual[2] - (l20 * y0) - (l21 * y1)) / l22;
    const float value = (y0 * y0) + (y1 * y1) + (y2 * y2);

    if (!isfinite(value) || (value < 0.0f))
        return false;

    *nis = value;
    return true;
}

static uint8_t current_hard_fault_mask(const imu_selector_input_t *input)
{
    uint8_t mask = 0U;

    for (uint32_t lane = 0U; lane < IMU_SELECTOR_LANE_COUNT; ++lane) {
        if (input->lane[lane].hard_fault_flags != 0U)
            mask |= IMU_SELECTOR_LANE_MASK(lane);
    }
    return mask;
}

static uint8_t numeric_valid_mask(const imu_selector_input_t *input,
                                  float covariance_floor_rad2)
{
    uint8_t mask = 0U;

    for (uint32_t lane = 0U; lane < IMU_SELECTOR_LANE_COUNT; ++lane) {
        if (lane_numeric_is_valid(&input->lane[lane], covariance_floor_rad2))
            mask |= IMU_SELECTOR_LANE_MASK(lane);
    }
    return mask;
}

static void update_residual_counters(imu_selector_t *selector,
                                     bool residual_valid,
                                     float residual_nis)
{
    if (!residual_valid) {
        selector->mismatch_streak = 0U;
        selector->clear_streak = 0U;
        selector->hint_streak[0] = 0U;
        selector->hint_streak[1] = 0U;
        return;
    }

    if (residual_nis > selector->config.nis_enter_threshold) {
        selector->mismatch_streak = increment_saturated(selector->mismatch_streak);
        selector->clear_streak = 0U;
    } else if (residual_nis < selector->config.nis_clear_threshold) {
        selector->clear_streak = increment_saturated(selector->clear_streak);
        selector->mismatch_streak = 0U;
        selector->hint_streak[0] = 0U;
        selector->hint_streak[1] = 0U;
    } else {
        selector->mismatch_streak = 0U;
        selector->clear_streak = 0U;
        selector->hint_streak[0] = 0U;
        selector->hint_streak[1] = 0U;
    }
}

static void update_recovery(imu_selector_t *selector,
                            uint8_t hard_mask,
                            uint8_t numeric_mask,
                            bool residual_enabled,
                            bool residual_clear)
{
    for (uint32_t lane = 0U; lane < IMU_SELECTOR_LANE_COUNT; ++lane) {
        const uint8_t lane_mask = IMU_SELECTOR_LANE_MASK(lane);
        const uint8_t other_mask = IMU_SELECTOR_LANE_MASK(1U - lane);
        const bool hard_latched = (selector->hard_latched_mask & lane_mask) != 0U;
        const bool soft_latched = (selector->soft_latched_mask & lane_mask) != 0U;

        if (!hard_latched && !soft_latched) {
            selector->recovery_streak[lane] = 0U;
            continue;
        }

        const bool lane_locally_clear = ((hard_mask & lane_mask) == 0U) &&
                                        ((numeric_mask & lane_mask) != 0U);
        const bool other_numeric = (numeric_mask & other_mask) != 0U;
        const bool cross_check_clear = residual_enabled && residual_clear;
        const bool recovery_evidence = hard_latched
                                           ? lane_locally_clear &&
                                                 (!other_numeric || cross_check_clear)
                                           : lane_locally_clear && other_numeric &&
                                                 cross_check_clear;

        if (!recovery_evidence) {
            selector->recovery_streak[lane] = 0U;
            continue;
        }

        selector->recovery_streak[lane] =
            increment_saturated(selector->recovery_streak[lane]);
        if (selector->recovery_streak[lane] >=
            selector->config.isolated_recovery_windows) {
            selector->hard_latched_mask &= (uint8_t)~lane_mask;
            selector->soft_latched_mask &= (uint8_t)~lane_mask;
            selector->recovery_streak[lane] = 0U;
        }
    }
}

static void update_external_isolation(imu_selector_t *selector,
                                      const imu_selector_input_t *input,
                                      uint8_t numeric_mask,
                                      bool residual_high)
{
    if (!residual_high) {
        selector->hint_streak[0] = 0U;
        selector->hint_streak[1] = 0U;
        return;
    }

    uint32_t hinted_lane = IMU_SELECTOR_LANE_COUNT;
    if (input->isolation_hint == IMU_SELECTOR_HINT_LANE_0_BAD)
        hinted_lane = 0U;
    else if (input->isolation_hint == IMU_SELECTOR_HINT_LANE_1_BAD)
        hinted_lane = 1U;

    if (hinted_lane >= IMU_SELECTOR_LANE_COUNT) {
        selector->hint_streak[0] = 0U;
        selector->hint_streak[1] = 0U;
        return;
    }

    const uint32_t other_lane = 1U - hinted_lane;
    const uint8_t hinted_mask = IMU_SELECTOR_LANE_MASK(hinted_lane);
    const uint8_t other_mask = IMU_SELECTOR_LANE_MASK(other_lane);
    const uint8_t isolated_mask = selector->hard_latched_mask |
                                  selector->soft_latched_mask;

    selector->hint_streak[other_lane] = 0U;
    if (((numeric_mask & hinted_mask) == 0U) ||
        ((numeric_mask & other_mask) == 0U) ||
        ((isolated_mask & other_mask) != 0U)) {
        selector->hint_streak[hinted_lane] = 0U;
        return;
    }

    selector->hint_streak[hinted_lane] =
        increment_saturated(selector->hint_streak[hinted_lane]);
    if (selector->hint_streak[hinted_lane] >=
        selector->config.soft_fault_confirm_windows) {
        selector->soft_latched_mask |= hinted_mask;
        selector->hint_streak[hinted_lane] = 0U;
    }
}

static imu_selector_lane_t choose_output_lane(const imu_selector_t *selector,
                                              uint8_t usable_mask)
{
    if ((selector->selected_lane != IMU_SELECTOR_LANE_NONE) &&
        ((usable_mask & IMU_SELECTOR_LANE_MASK(selector->selected_lane)) != 0U))
        return selector->selected_lane;

    const imu_selector_lane_t other_lane =
        (selector->selected_lane == IMU_SELECTOR_LANE_0) ? IMU_SELECTOR_LANE_1
                                                         : IMU_SELECTOR_LANE_0;
    if ((selector->selected_lane != IMU_SELECTOR_LANE_NONE) &&
        ((usable_mask & IMU_SELECTOR_LANE_MASK(other_lane)) != 0U))
        return other_lane;

    if ((usable_mask & IMU_SELECTOR_LANE_MASK(selector->preferred_lane)) != 0U)
        return selector->preferred_lane;

    const imu_selector_lane_t alternate =
        (selector->preferred_lane == IMU_SELECTOR_LANE_0) ? IMU_SELECTOR_LANE_1
                                                          : IMU_SELECTOR_LANE_0;
    if ((usable_mask & IMU_SELECTOR_LANE_MASK(alternate)) != 0U)
        return alternate;

    return IMU_SELECTOR_LANE_NONE;
}

void imu_selector_default_config(imu_selector_config_t *config)
{
    if (config == NULL)
        return;

    config->nis_enter_threshold = IMU_SELECTOR_CHI_SQUARE_3DOF_99;
    config->nis_clear_threshold = IMU_SELECTOR_CHI_SQUARE_3DOF_95;
    config->covariance_floor_rad2 = 1.0e-12f;
    config->suspect_enter_windows = 3U;
    config->ambiguous_enter_windows = 8U;
    config->soft_fault_confirm_windows = 8U;
    config->clear_windows = 20U;
    config->isolated_recovery_windows = 40U;
}

bool imu_selector_init(imu_selector_t *selector,
                       const imu_selector_config_t *config,
                       imu_selector_lane_t preferred_lane)
{
    if ((selector == NULL) || !config_is_valid(config) ||
        ((preferred_lane != IMU_SELECTOR_LANE_0) &&
         (preferred_lane != IMU_SELECTOR_LANE_1)))
        return false;

    memset(selector, 0, sizeof(*selector));
    selector->config = *config;
    selector->state = IMU_SELECTOR_SUSPECT;
    selector->preferred_lane = preferred_lane;
    selector->selected_lane = preferred_lane;
    selector->initialized = true;
    return true;
}

bool imu_selector_update(imu_selector_t *selector,
                         const imu_selector_input_t *input,
                         imu_selector_result_t *result)
{
    if ((selector == NULL) || !selector->initialized || (input == NULL) ||
        (result == NULL))
        return false;

    memset(result, 0, sizeof(*result));
    result->residual_nis = NAN;

    const imu_selector_lane_t previous_selection = selector->selected_lane;
    const imu_selector_state_t previous_state = selector->state;
    const uint8_t hard_mask = current_hard_fault_mask(input);
    const uint8_t numeric_mask =
        numeric_valid_mask(input, selector->config.covariance_floor_rad2);

    selector->hard_latched_mask |= hard_mask;

    bool residual_valid = false;
    float residual_nis = NAN;
    float residual[3] = {0.0f, 0.0f, 0.0f};
    if (input->residual_enabled && (numeric_mask == 0x03U)) {
        residual_valid = calculate_nis(input,
                                       selector->config.covariance_floor_rad2,
                                       residual,
                                       &residual_nis);
        update_residual_counters(selector, residual_valid, residual_nis);
    } else if (input->residual_enabled) {
        update_residual_counters(selector, false, NAN);
    }

    const bool residual_high = residual_valid &&
                               (residual_nis > selector->config.nis_enter_threshold);
    const bool residual_clear = residual_valid &&
                                (residual_nis < selector->config.nis_clear_threshold);

    update_recovery(selector,
                    hard_mask,
                    numeric_mask,
                    input->residual_enabled,
                    residual_clear);
    if (input->residual_enabled)
        update_external_isolation(selector, input, numeric_mask, residual_high);

    const uint8_t isolated_mask = selector->hard_latched_mask |
                                  selector->soft_latched_mask;
    const uint8_t usable_mask = numeric_mask & (uint8_t)~isolated_mask;
    selector->selected_lane = choose_output_lane(selector, usable_mask);

    const uint8_t invalid_mask = (uint8_t)(0x03U & (uint8_t)~numeric_mask);
    if ((isolated_mask != 0U) ||
        (selector->selected_lane == IMU_SELECTOR_LANE_NONE)) {
        selector->state = IMU_SELECTOR_FAULT;
    } else if (selector->mismatch_streak >=
               selector->config.ambiguous_enter_windows) {
        selector->state = IMU_SELECTOR_AMBIGUOUS;
    } else if (((previous_state == IMU_SELECTOR_AMBIGUOUS) &&
                (selector->clear_streak < selector->config.clear_windows)) ||
               (selector->mismatch_streak >=
                selector->config.suspect_enter_windows) ||
               (invalid_mask != 0U) ||
               ((previous_state == IMU_SELECTOR_FAULT) &&
                (selector->clear_streak < selector->config.clear_windows)) ||
               ((previous_state == IMU_SELECTOR_SUSPECT) &&
                (selector->clear_streak < selector->config.clear_windows))) {
        selector->state = (previous_state == IMU_SELECTOR_AMBIGUOUS)
                              ? IMU_SELECTOR_AMBIGUOUS
                              : IMU_SELECTOR_SUSPECT;
    } else {
        selector->state = IMU_SELECTOR_HEALTHY;
    }

    uint8_t suspect_mask = isolated_mask | invalid_mask;
    if ((selector->state == IMU_SELECTOR_SUSPECT) ||
        (selector->state == IMU_SELECTOR_AMBIGUOUS))
        suspect_mask |= 0x03U;
    if (selector->hint_streak[0] > 0U)
        suspect_mask |= IMU_SELECTOR_LANE_MASK(0U);
    if (selector->hint_streak[1] > 0U)
        suspect_mask |= IMU_SELECTOR_LANE_MASK(1U);

    uint32_t reasons = IMU_SELECTOR_REASON_NONE;
    if ((hard_mask & IMU_SELECTOR_LANE_MASK(0U)) != 0U)
        reasons |= IMU_SELECTOR_REASON_LANE_0_HARD;
    if ((hard_mask & IMU_SELECTOR_LANE_MASK(1U)) != 0U)
        reasons |= IMU_SELECTOR_REASON_LANE_1_HARD;
    if ((invalid_mask & IMU_SELECTOR_LANE_MASK(0U)) != 0U)
        reasons |= IMU_SELECTOR_REASON_LANE_0_INPUT;
    if ((invalid_mask & IMU_SELECTOR_LANE_MASK(1U)) != 0U)
        reasons |= IMU_SELECTOR_REASON_LANE_1_INPUT;
    if (input->residual_enabled && (numeric_mask == 0x03U) && !residual_valid)
        reasons |= IMU_SELECTOR_REASON_NIS_INVALID;
    if (residual_high)
        reasons |= IMU_SELECTOR_REASON_NIS_HIGH;
    if ((input->isolation_hint != IMU_SELECTOR_HINT_NONE) && residual_high)
        reasons |= IMU_SELECTOR_REASON_EXTERNAL_HINT;
    if (selector->selected_lane == IMU_SELECTOR_LANE_NONE)
        reasons |= IMU_SELECTOR_REASON_NO_OUTPUT;

    result->state = selector->state;
    result->selected_lane = selector->selected_lane;
    result->residual_rad[0] = residual[0];
    result->residual_rad[1] = residual[1];
    result->residual_rad[2] = residual[2];
    result->residual_nis = residual_nis;
    result->residual_valid = residual_valid;
    result->selection_changed = previous_selection != selector->selected_lane;
    result->hard_fault_mask = hard_mask;
    result->isolated_mask = isolated_mask;
    result->usable_mask = usable_mask;
    result->suspect_mask = suspect_mask;
    result->reason_flags = reasons;
    result->mismatch_streak = selector->mismatch_streak;
    result->clear_streak = selector->clear_streak;
    return true;
}
