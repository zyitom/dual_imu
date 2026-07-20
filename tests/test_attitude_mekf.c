#include "attitude_mekf.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_PI_F      (3.14159265358979323846f)
#define TEST_GRAVITY   (9.80665f)
#define TEST_DT_S      (0.0025f)
#define TEST_NOISE_SAMPLE_COUNT (8000U)

static unsigned int failure_count;

#define TEST_EXPECT(condition)                                                        \
    do                                                                                \
    {                                                                                 \
        if (!(condition))                                                             \
        {                                                                             \
            (void)fprintf(stderr, "%s:%d: expectation failed: %s\n",                 \
                          __FILE__, __LINE__, #condition);                             \
            failure_count++;                                                          \
        }                                                                             \
    } while (0)

static float degrees_to_radians(float degrees)
{
    return degrees * TEST_PI_F / 180.0f;
}

static float deterministic_signed_unit_noise(uint32_t *state)
{
    *state = (UINT32_C(1664525) * *state) + UINT32_C(1013904223);
    return ((float)((*state >> 8) & UINT32_C(0x00FFFFFF)) / 8388608.0f) -
           1.0f;
}

static float wrap_pi(float angle)
{
    while (angle > TEST_PI_F)
    {
        angle -= 2.0f * TEST_PI_F;
    }
    while (angle < -TEST_PI_F)
    {
        angle += 2.0f * TEST_PI_F;
    }
    return angle;
}

static float quaternion_distance(const float lhs[4], const float rhs[4])
{
    float dot = fabsf((lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) +
                      (lhs[2] * rhs[2]) + (lhs[3] * rhs[3]));
    dot = fminf(1.0f, fmaxf(-1.0f, dot));
    return 2.0f * acosf(dot);
}

static void quaternion_from_euler(float roll,
                                  float pitch,
                                  float yaw,
                                  float quaternion[4])
{
    const float cosine_roll = cosf(0.5f * roll);
    const float sine_roll = sinf(0.5f * roll);
    const float cosine_pitch = cosf(0.5f * pitch);
    const float sine_pitch = sinf(0.5f * pitch);
    const float cosine_yaw = cosf(0.5f * yaw);
    const float sine_yaw = sinf(0.5f * yaw);

    quaternion[0] = (cosine_yaw * cosine_pitch * cosine_roll) +
                    (sine_yaw * sine_pitch * sine_roll);
    quaternion[1] = (cosine_yaw * cosine_pitch * sine_roll) -
                    (sine_yaw * sine_pitch * cosine_roll);
    quaternion[2] = (cosine_yaw * sine_pitch * cosine_roll) +
                    (sine_yaw * cosine_pitch * sine_roll);
    quaternion[3] = (sine_yaw * cosine_pitch * cosine_roll) -
                    (cosine_yaw * sine_pitch * sine_roll);
}

static void static_specific_force(const float quaternion[4], float specific_force[3])
{
    specific_force[0] = -TEST_GRAVITY *
                        (2.0f * ((quaternion[1] * quaternion[3]) -
                                 (quaternion[0] * quaternion[2])));
    specific_force[1] = -TEST_GRAVITY *
                        (2.0f * ((quaternion[0] * quaternion[1]) +
                                 (quaternion[2] * quaternion[3])));
    specific_force[2] = -TEST_GRAVITY *
                        ((quaternion[0] * quaternion[0]) -
                         (quaternion[1] * quaternion[1]) -
                         (quaternion[2] * quaternion[2]) +
                         (quaternion[3] * quaternion[3]));
}

static void test_initialization_and_validation(void)
{
    attitude_mekf_t filter;
    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
    TEST_EXPECT(fabsf(filter.q[0] - 1.0f) < 1.0e-6f);

    attitude_mekf_config_t bad_config;
    attitude_mekf_default_config(&bad_config);
    bad_config.accel_nis_gate = -1.0f;
    TEST_EXPECT(!attitude_mekf_init(&filter, &bad_config));
    TEST_EXPECT(!filter.initialized);
}

static void test_delta_angle_propagation(void)
{
    attitude_mekf_t filter;
    const float yaw_rate = degrees_to_radians(90.0f);
    const float delta_angle[3] = {0.0f, 0.0f, yaw_rate * TEST_DT_S};
    float euler[3];

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    for (unsigned int sample = 0U; sample < 400U; ++sample)
    {
        TEST_EXPECT(attitude_mekf_propagate_delta(&filter, delta_angle, TEST_DT_S));
    }
    TEST_EXPECT(attitude_mekf_get_euler(&filter, euler));
    TEST_EXPECT(fabsf(wrap_pi(euler[2] - (0.5f * TEST_PI_F))) < 2.0e-4f);
    TEST_EXPECT(fabsf(euler[0]) < 1.0e-5f);
    TEST_EXPECT(fabsf(euler[1]) < 1.0e-5f);
    TEST_EXPECT(filter.diagnostics.propagation_count == 400U);
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
}

static void test_accel_seed_coordinate_convention(void)
{
    attitude_mekf_t filter;
    const float expected_euler[3] = {
        degrees_to_radians(20.0f),
        degrees_to_radians(-10.0f),
        degrees_to_radians(35.0f),
    };
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    float true_quaternion[4];
    float specific_force[3];
    float estimated_euler[3];

    quaternion_from_euler(expected_euler[0], expected_euler[1], expected_euler[2],
                          true_quaternion);
    static_specific_force(true_quaternion, specific_force);

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_seed_from_accel(&filter, specific_force,
                                              expected_euler[2], zero_bias));
    TEST_EXPECT(attitude_mekf_get_euler(&filter, estimated_euler));
    for (unsigned int axis = 0U; axis < 3U; ++axis)
    {
        TEST_EXPECT(fabsf(wrap_pi(estimated_euler[axis] - expected_euler[axis])) <
                    2.0e-5f);
    }
}

static void test_tilt_and_bias_convergence(void)
{
    attitude_mekf_t filter;
    float initial_quaternion[4];
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const float true_specific_force[3] = {0.0f, 0.0f, -TEST_GRAVITY};
    const float injected_bias = 0.02f;
    const float measured_delta[3] = {injected_bias * TEST_DT_S, 0.0f, 0.0f};
    float euler[3];
    float estimated_bias[3];

    quaternion_from_euler(degrees_to_radians(8.0f), 0.0f, 0.0f,
                          initial_quaternion);
    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_reset(&filter, initial_quaternion, zero_bias));

    for (unsigned int sample = 0U; sample < 8000U; ++sample)
    {
        TEST_EXPECT(attitude_mekf_propagate_delta(&filter, measured_delta, TEST_DT_S));
        const attitude_mekf_accel_result_t result =
            attitude_mekf_update_accel(&filter, true_specific_force, 1.0f);
        TEST_EXPECT(result == ATTITUDE_MEKF_ACCEL_ACCEPTED);
    }

    TEST_EXPECT(attitude_mekf_get_euler(&filter, euler));
    TEST_EXPECT(attitude_mekf_get_bias(&filter, estimated_bias));
    TEST_EXPECT(fabsf(euler[0]) < degrees_to_radians(0.05f));
    TEST_EXPECT(fabsf(estimated_bias[0] - injected_bias) < 2.0e-4f);
    TEST_EXPECT(fabsf(estimated_bias[1]) < 1.0e-4f);
    TEST_EXPECT(fabsf(estimated_bias[2]) < 1.0e-4f);
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
}

static void test_gravity_update_at_arbitrary_attitude(void)
{
    attitude_mekf_t filter;
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    float true_quaternion[4];
    float initial_quaternion[4];
    float true_specific_force[3];
    float estimated_specific_force[3];

    quaternion_from_euler(degrees_to_radians(25.0f), degrees_to_radians(-15.0f),
                          degrees_to_radians(50.0f), true_quaternion);
    quaternion_from_euler(degrees_to_radians(32.0f), degrees_to_radians(-20.0f),
                          degrees_to_radians(50.0f), initial_quaternion);
    static_specific_force(true_quaternion, true_specific_force);

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_reset(&filter, initial_quaternion, zero_bias));
    for (unsigned int update = 0U; update < 200U; ++update)
    {
        TEST_EXPECT(attitude_mekf_update_accel(&filter, true_specific_force, 1.0f) ==
                    ATTITUDE_MEKF_ACCEL_ACCEPTED);
    }

    static_specific_force(filter.q, estimated_specific_force);
    float direction_error_sq = 0.0f;
    for (unsigned int axis = 0U; axis < 3U; ++axis)
    {
        const float difference =
            (estimated_specific_force[axis] - true_specific_force[axis]) / TEST_GRAVITY;
        direction_error_sq += difference * difference;
    }
    TEST_EXPECT(sqrtf(direction_error_sq) < 2.0e-4f);
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
}

/*
 * A specific force whose magnitude is off gravity but whose direction still
 * carries gravity must reach the measurement update, down-weighted, instead of
 * being dropped. Dropping it is what let a sustained high-rate sweep run tilt
 * open-loop; the seed path, which writes tilt straight from the vector, must
 * still refuse the same input.
 */
static void test_off_norm_accel_updates_but_does_not_seed(void)
{
    attitude_mekf_t filter;
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const float static_force[3] = {0.0f, 0.0f, -TEST_GRAVITY};
    const float zero_delta[3] = {0.0f, 0.0f, 0.0f};
    /* 1.5 g along the level down axis: a centrifugal/linear disturbance that
     * has not yet swamped the gravity direction. */
    const float inflated_force[3] = {0.0f, 0.0f, -1.5f * TEST_GRAVITY};

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    for (unsigned int sample = 0U; sample < 400U; ++sample)
    {
        TEST_EXPECT(attitude_mekf_propagate_delta(&filter, zero_delta, TEST_DT_S));
        TEST_EXPECT(attitude_mekf_update_accel(&filter, static_force, 1.0f) ==
                    ATTITUDE_MEKF_ACCEL_ACCEPTED);
    }

    TEST_EXPECT(attitude_mekf_update_accel(&filter, inflated_force, 1.0f) ==
                ATTITUDE_MEKF_ACCEL_ACCEPTED);
    /* Accepted, but the norm excursion must have inflated the measurement
     * variance far above the nominal weight rather than trusting it. */
    TEST_EXPECT(filter.diagnostics.last_accel_variance_scale > 50.0f);

    /* Same vector through the seeding path is still refused: there is no
     * covariance there to discount a contaminated tilt. */
    TEST_EXPECT(!attitude_mekf_seed_from_accel(&filter, inflated_force, 0.0f,
                                               zero_bias));

    /* Beyond the update gate the direction is more disturbance than gravity. */
    const float overwhelming_force[3] = {0.0f, 0.0f, -3.0f * TEST_GRAVITY};
    TEST_EXPECT(attitude_mekf_update_accel(&filter, overwhelming_force, 1.0f) ==
                ATTITUDE_MEKF_ACCEL_REJECTED_NORM);
}

static void test_accel_rejection_is_transactional(void)
{
    attitude_mekf_t filter;
    const float static_force[3] = {0.0f, 0.0f, -TEST_GRAVITY};
    const float gross_force[3] = {20.0f, 0.0f, -TEST_GRAVITY};
    const float zero_delta[3] = {0.0f, 0.0f, 0.0f};

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    for (unsigned int sample = 0U; sample < 400U; ++sample)
    {
        TEST_EXPECT(attitude_mekf_propagate_delta(&filter, zero_delta, TEST_DT_S));
        TEST_EXPECT(attitude_mekf_update_accel(&filter, static_force, 1.0f) ==
                    ATTITUDE_MEKF_ACCEL_ACCEPTED);
    }

    float saved_quaternion[4];
    float saved_bias[3];
    float saved_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    memcpy(saved_quaternion, filter.q, sizeof(saved_quaternion));
    memcpy(saved_bias, filter.gyro_bias_rad_s, sizeof(saved_bias));
    memcpy(saved_covariance, filter.covariance, sizeof(saved_covariance));

    TEST_EXPECT(attitude_mekf_update_accel(&filter, gross_force, 1.0f) ==
                ATTITUDE_MEKF_ACCEL_REJECTED_NORM);
    TEST_EXPECT(memcmp(saved_quaternion, filter.q, sizeof(saved_quaternion)) == 0);
    TEST_EXPECT(memcmp(saved_bias, filter.gyro_bias_rad_s, sizeof(saved_bias)) == 0);
    TEST_EXPECT(memcmp(saved_covariance, filter.covariance, sizeof(saved_covariance)) == 0);

    const float lateral_force[3] = {
        TEST_GRAVITY * sinf(degrees_to_radians(15.0f)),
        0.0f,
        -TEST_GRAVITY * cosf(degrees_to_radians(15.0f)),
    };
    TEST_EXPECT(attitude_mekf_update_accel(&filter, lateral_force, 1.0f) ==
                ATTITUDE_MEKF_ACCEL_REJECTED_NIS);
    TEST_EXPECT(memcmp(saved_quaternion, filter.q, sizeof(saved_quaternion)) == 0);
    TEST_EXPECT(memcmp(saved_covariance, filter.covariance, sizeof(saved_covariance)) == 0);

    const float invalid_force[3] = {NAN, 0.0f, -TEST_GRAVITY};
    TEST_EXPECT(attitude_mekf_update_accel(&filter, invalid_force, 1.0f) ==
                ATTITUDE_MEKF_ACCEL_REJECTED_INVALID_INPUT);
    TEST_EXPECT(memcmp(saved_quaternion, filter.q, sizeof(saved_quaternion)) == 0);
    TEST_EXPECT(memcmp(saved_covariance, filter.covariance, sizeof(saved_covariance)) == 0);
}

static void test_inflated_covariance_accel_correction_is_step_limited(void)
{
    attitude_mekf_t filter;
    const float contamination_angle = degrees_to_radians(17.0f);
    const float contaminated_force[3] = {
        0.0f,
        -TEST_GRAVITY * sinf(contamination_angle),
        -TEST_GRAVITY * cosf(contamination_angle),
    };

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_mark_rotation_unobserved(
        &filter, degrees_to_radians(30.0f)));
    float before[4];
    memcpy(before, filter.q, sizeof(before));

    TEST_EXPECT(attitude_mekf_update_accel(
                    &filter, contaminated_force, 1.0f) ==
                ATTITUDE_MEKF_ACCEL_ACCEPTED);
    const float applied_step = quaternion_distance(before, filter.q);
    TEST_EXPECT(applied_step <=
                filter.config.max_accel_correction_step_rad + 1.0e-5f);
    TEST_EXPECT(applied_step >=
                0.9f * filter.config.max_accel_correction_step_rad);
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
}

static void test_yaw_unobservability_and_propagation_rejection(void)
{
    attitude_mekf_t filter;
    float initial_quaternion[4];
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const float static_force[3] = {0.0f, 0.0f, -TEST_GRAVITY};
    const float yaw_bias = 0.01f;
    const float yaw_delta[3] = {0.0f, 0.0f, yaw_bias * TEST_DT_S};
    float euler[3];
    float estimated_bias[3];

    quaternion_from_euler(0.0f, 0.0f, degrees_to_radians(30.0f),
                          initial_quaternion);
    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_reset(&filter, initial_quaternion, zero_bias));
    const float initial_yaw_variance = filter.covariance[2][2];

    for (unsigned int sample = 0U; sample < 1600U; ++sample)
    {
        TEST_EXPECT(attitude_mekf_propagate_delta(&filter, yaw_delta, TEST_DT_S));
        TEST_EXPECT(attitude_mekf_update_accel(&filter, static_force, 1.0f) ==
                    ATTITUDE_MEKF_ACCEL_ACCEPTED);
    }
    TEST_EXPECT(attitude_mekf_get_euler(&filter, euler));
    TEST_EXPECT(attitude_mekf_get_bias(&filter, estimated_bias));
    TEST_EXPECT(fabsf(wrap_pi(euler[2] -
                              (degrees_to_radians(30.0f) + (yaw_bias * 4.0f)))) <
                2.0e-4f);
    TEST_EXPECT(fabsf(estimated_bias[2]) < 1.0e-7f);
    TEST_EXPECT(filter.covariance[2][2] >= initial_yaw_variance);

    float saved_quaternion[4];
    float saved_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    memcpy(saved_quaternion, filter.q, sizeof(saved_quaternion));
    memcpy(saved_covariance, filter.covariance, sizeof(saved_covariance));
    const float invalid_delta[3] = {NAN, 0.0f, 0.0f};
    TEST_EXPECT(!attitude_mekf_propagate_delta(&filter, invalid_delta, TEST_DT_S));
    TEST_EXPECT(memcmp(saved_quaternion, filter.q, sizeof(saved_quaternion)) == 0);
    TEST_EXPECT(memcmp(saved_covariance, filter.covariance, sizeof(saved_covariance)) == 0);
}

static void test_gravity_axis_bias_is_unobservable_at_tilt(void)
{
    attitude_mekf_t filter;
    float true_quaternion[4];
    float static_force[3];
    float estimated_bias[3];
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const float bias_magnitude = 0.02f;

    quaternion_from_euler(degrees_to_radians(-18.0f),
                          degrees_to_radians(13.0f),
                          degrees_to_radians(27.0f),
                          true_quaternion);
    static_specific_force(true_quaternion, static_force);
    const float down_body[3] = {
        -static_force[0] / TEST_GRAVITY,
        -static_force[1] / TEST_GRAVITY,
        -static_force[2] / TEST_GRAVITY,
    };
    const float measured_delta[3] = {
        bias_magnitude * down_body[0] * TEST_DT_S,
        bias_magnitude * down_body[1] * TEST_DT_S,
        bias_magnitude * down_body[2] * TEST_DT_S,
    };

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_reset(&filter, true_quaternion, zero_bias));
    for (unsigned int sample = 0U; sample < 4000U; ++sample)
    {
        TEST_EXPECT(attitude_mekf_propagate_delta(&filter,
                                                  measured_delta,
                                                  TEST_DT_S));
        TEST_EXPECT(attitude_mekf_update_accel(&filter, static_force, 1.0f) ==
                    ATTITUDE_MEKF_ACCEL_ACCEPTED);
    }

    TEST_EXPECT(attitude_mekf_get_bias(&filter, estimated_bias));
    const float learned_gravity_axis_bias =
        (estimated_bias[0] * down_body[0]) +
        (estimated_bias[1] * down_body[1]) +
        (estimated_bias[2] * down_body[2]);
    TEST_EXPECT(fabsf(learned_gravity_axis_bias) < 1.0e-4f);
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
}

static void test_noisy_tilt_preserves_gravity_gauge_and_observable_bias(void)
{
    attitude_mekf_t filter;
    float true_quaternion[4];
    float static_force[3];
    float estimated_bias[3];
    float euler[3];
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const float gyro_noise_amplitude = degrees_to_radians(3.0f);
    const float accel_noise_amplitude = 0.03f;
    double gyro_noise_mean[3] = {0.0, 0.0, 0.0};
    double accel_noise_mean[3] = {0.0, 0.0, 0.0};
    uint32_t noise_state = UINT32_C(7);

    /* Subtract the finite-record mean so this stress sequence contains no
     * real constant rate or force that the filter could legitimately learn. */
    for (unsigned int sample = 0U; sample < TEST_NOISE_SAMPLE_COUNT; ++sample)
    {
        for (unsigned int axis = 0U; axis < 3U; ++axis)
        {
            gyro_noise_mean[axis] +=
                (double)gyro_noise_amplitude *
                (double)deterministic_signed_unit_noise(&noise_state);
            accel_noise_mean[axis] +=
                (double)accel_noise_amplitude *
                (double)deterministic_signed_unit_noise(&noise_state);
        }
    }
    for (unsigned int axis = 0U; axis < 3U; ++axis)
    {
        gyro_noise_mean[axis] /= (double)TEST_NOISE_SAMPLE_COUNT;
        accel_noise_mean[axis] /= (double)TEST_NOISE_SAMPLE_COUNT;
    }

    quaternion_from_euler(degrees_to_radians(-2.0f),
                          degrees_to_radians(1.5f),
                          0.0f,
                          true_quaternion);
    static_specific_force(true_quaternion, static_force);
    const float down_body[3] = {
        -static_force[0] / TEST_GRAVITY,
        -static_force[1] / TEST_GRAVITY,
        -static_force[2] / TEST_GRAVITY,
    };

    float observable_direction[3] = {
        1.0f - (down_body[0] * down_body[0]),
        -down_body[0] * down_body[1],
        -down_body[0] * down_body[2],
    };
    const float observable_direction_norm =
        sqrtf((observable_direction[0] * observable_direction[0]) +
              (observable_direction[1] * observable_direction[1]) +
              (observable_direction[2] * observable_direction[2]));
    const float observable_bias_magnitude = 0.02f;
    float observable_bias[3];
    for (unsigned int axis = 0U; axis < 3U; ++axis)
    {
        observable_direction[axis] /= observable_direction_norm;
        observable_bias[axis] =
            observable_bias_magnitude * observable_direction[axis];
    }

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_seed_from_accel(&filter, static_force,
                                              0.0f, zero_bias));
    noise_state = UINT32_C(7);
    for (unsigned int sample = 0U; sample < TEST_NOISE_SAMPLE_COUNT; ++sample)
    {
        float delta_angle[3];
        float noisy_force[3];
        for (unsigned int axis = 0U; axis < 3U; ++axis)
        {
            const float gyro_noise =
                (gyro_noise_amplitude *
                 deterministic_signed_unit_noise(&noise_state)) -
                (float)gyro_noise_mean[axis];
            const float accel_noise =
                (accel_noise_amplitude *
                 deterministic_signed_unit_noise(&noise_state)) -
                (float)accel_noise_mean[axis];
            delta_angle[axis] =
                (observable_bias[axis] + gyro_noise) * TEST_DT_S;
            noisy_force[axis] = static_force[axis] + accel_noise;
        }

        TEST_EXPECT(attitude_mekf_propagate_delta(&filter,
                                                  delta_angle,
                                                  TEST_DT_S));
        TEST_EXPECT(attitude_mekf_update_accel(&filter, noisy_force, 1.0f) ==
                    ATTITUDE_MEKF_ACCEL_ACCEPTED);
    }

    TEST_EXPECT(attitude_mekf_get_bias(&filter, estimated_bias));
    TEST_EXPECT(attitude_mekf_get_euler(&filter, euler));
    const float gravity_axis_bias =
        (estimated_bias[0] * down_body[0]) +
        (estimated_bias[1] * down_body[1]) +
        (estimated_bias[2] * down_body[2]);
    const float learned_observable_bias =
        (estimated_bias[0] * observable_direction[0]) +
        (estimated_bias[1] * observable_direction[1]) +
        (estimated_bias[2] * observable_direction[2]);
    TEST_EXPECT(fabsf(gravity_axis_bias) < 5.0e-4f);
    TEST_EXPECT(fabsf(learned_observable_bias - observable_bias_magnitude) <
                5.0e-4f);
    TEST_EXPECT(fabsf(wrap_pi(euler[2])) < degrees_to_radians(1.0f));
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
}

static void test_zaru_three_axis_bias_update(void)
{
    attitude_mekf_t filter;
    float initial_quaternion[4];
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const float measured_bias[3] = {0.012f, -0.007f, 0.025f};
    const float measured_delta[3] = {
        measured_bias[0] * TEST_DT_S,
        measured_bias[1] * TEST_DT_S,
        measured_bias[2] * TEST_DT_S,
    };
    const float measurement_covariance[3][3] = {
        {1.0e-6f, 2.0e-7f, 0.0f},
        {2.0e-7f, 1.2e-6f, 1.0e-7f},
        {0.0f, 1.0e-7f, 8.0e-7f},
    };
    float estimated_bias[3];
    float euler[3];

    quaternion_from_euler(0.0f, 0.0f, degrees_to_radians(30.0f),
                          initial_quaternion);
    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_reset(&filter, initial_quaternion, zero_bias));

    for (unsigned int sample = 0U; sample < 400U; ++sample)
    {
        TEST_EXPECT(attitude_mekf_propagate_delta(&filter, measured_delta, TEST_DT_S));
        if ((sample % 10U) == 0U)
        {
            TEST_EXPECT(attitude_mekf_update_zero_rate(
                            &filter, measured_bias, measurement_covariance) ==
                        ATTITUDE_MEKF_ZARU_ACCEPTED);
        }
    }

    TEST_EXPECT(attitude_mekf_get_bias(&filter, estimated_bias));
    for (unsigned int axis = 0U; axis < 3U; ++axis)
    {
        TEST_EXPECT(fabsf(estimated_bias[axis] - measured_bias[axis]) < 2.0e-5f);
    }
    TEST_EXPECT(attitude_mekf_get_euler(&filter, euler));
    TEST_EXPECT(fabsf(wrap_pi(euler[2] - degrees_to_radians(30.0f))) < 2.0e-4f);
    TEST_EXPECT(filter.diagnostics.zaru_update_count == 40U);
    TEST_EXPECT(filter.diagnostics.zaru_accept_count == 40U);
    TEST_EXPECT(filter.diagnostics.last_zaru_result == ATTITUDE_MEKF_ZARU_ACCEPTED);
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
}

static void test_bounded_zaru_clips_target_and_limits_global_slew(void)
{
    attitude_mekf_t filter;
    const float measured_rate[3] = {
        degrees_to_radians(2.0f),
        degrees_to_radians(-1.0f),
        degrees_to_radians(0.25f),
    };
    const float measured_delta[3] = {
        measured_rate[0] * TEST_DT_S,
        measured_rate[1] * TEST_DT_S,
        measured_rate[2] * TEST_DT_S,
    };
    const float covariance[3][3] = {
        {1.0e-7f, 0.0f, 0.0f},
        {0.0f, 1.0e-7f, 0.0f},
        {0.0f, 0.0f, 1.0e-7f},
    };
    const float target_limit = degrees_to_radians(0.5f);
    const float maximum_bias_step =
        degrees_to_radians(0.1f) * TEST_DT_S;
    float bounded_target[3];
    float bias_before[3];
    float bias_after[3];

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_propagate_delta(&filter,
                                              measured_delta,
                                              TEST_DT_S));
    TEST_EXPECT(attitude_mekf_get_bias(&filter, bias_before));
    TEST_EXPECT(attitude_mekf_update_zero_rate_bounded(
                    &filter,
                    measured_rate,
                    covariance,
                    target_limit,
                    maximum_bias_step,
                    bounded_target) == ATTITUDE_MEKF_ZARU_ACCEPTED);
    TEST_EXPECT(attitude_mekf_get_bias(&filter, bias_after));

    TEST_EXPECT(fabsf(bounded_target[0] - target_limit) < 1.0e-8f);
    TEST_EXPECT(fabsf(bounded_target[1] + target_limit) < 1.0e-8f);
    TEST_EXPECT(fabsf(bounded_target[2] - measured_rate[2]) < 1.0e-8f);
    float bias_step_sq = 0.0f;
    for (unsigned int axis = 0U; axis < 3U; ++axis)
    {
        const float step = bias_after[axis] - bias_before[axis];
        bias_step_sq += step * step;
    }
    const float bias_step = sqrtf(bias_step_sq);
    TEST_EXPECT(bias_step <= maximum_bias_step + 1.0e-9f);
    TEST_EXPECT(bias_step >= 0.99f * maximum_bias_step);
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
}

static void test_bounded_zaru_clips_about_existing_bias(void)
{
    attitude_mekf_t filter;
    const float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float initial_bias[3] = {
        degrees_to_radians(0.8f),
        degrees_to_radians(-0.7f),
        0.0f,
    };
    const float measured_rate[3] = {
        degrees_to_radians(2.0f),
        degrees_to_radians(-0.9f),
        0.0f,
    };
    const float covariance[3][3] = {
        {1.0e-7f, 0.0f, 0.0f},
        {0.0f, 1.0e-7f, 0.0f},
        {0.0f, 0.0f, 1.0e-7f},
    };
    const float target_limit = degrees_to_radians(0.5f);
    float bounded_target[3];

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    TEST_EXPECT(attitude_mekf_reset(&filter, quaternion, initial_bias));
    TEST_EXPECT(attitude_mekf_update_zero_rate_bounded(
                    &filter,
                    measured_rate,
                    covariance,
                    target_limit,
                    degrees_to_radians(0.1f) * TEST_DT_S,
                    bounded_target) == ATTITUDE_MEKF_ZARU_ACCEPTED);
    TEST_EXPECT(fabsf(bounded_target[0] - degrees_to_radians(1.3f)) <
                1.0e-7f);
    TEST_EXPECT(fabsf(bounded_target[1] - measured_rate[1]) < 1.0e-7f);
    TEST_EXPECT(fabsf(bounded_target[2]) < 1.0e-7f);
}

static void test_zaru_rejection_is_transactional(void)
{
    attitude_mekf_t filter;
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    const float measurement_covariance[3][3] = {
        {1.0e-7f, 0.0f, 0.0f},
        {0.0f, 1.0e-7f, 0.0f},
        {0.0f, 0.0f, 1.0e-7f},
    };

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    for (unsigned int update = 0U; update < 20U; ++update)
    {
        TEST_EXPECT(attitude_mekf_update_zero_rate(
                        &filter, zero_rate, measurement_covariance) ==
                    ATTITUDE_MEKF_ZARU_ACCEPTED);
    }

    float saved_quaternion[4];
    float saved_bias[3];
    float saved_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    memcpy(saved_quaternion, filter.q, sizeof(saved_quaternion));
    memcpy(saved_bias, filter.gyro_bias_rad_s, sizeof(saved_bias));
    memcpy(saved_covariance, filter.covariance, sizeof(saved_covariance));

    const float moving_rate[3] = {0.4f, 0.0f, 0.0f};
    TEST_EXPECT(attitude_mekf_update_zero_rate(
                    &filter, moving_rate, measurement_covariance) ==
                ATTITUDE_MEKF_ZARU_REJECTED_NIS);
    TEST_EXPECT(memcmp(saved_quaternion, filter.q, sizeof(saved_quaternion)) == 0);
    TEST_EXPECT(memcmp(saved_bias, filter.gyro_bias_rad_s, sizeof(saved_bias)) == 0);
    TEST_EXPECT(memcmp(saved_covariance, filter.covariance, sizeof(saved_covariance)) == 0);

    const float nonsymmetric_covariance[3][3] = {
        {1.0e-6f, 5.0e-7f, 0.0f},
        {0.0f, 1.0e-6f, 0.0f},
        {0.0f, 0.0f, 1.0e-6f},
    };
    TEST_EXPECT(attitude_mekf_update_zero_rate(
                    &filter, zero_rate, nonsymmetric_covariance) ==
                ATTITUDE_MEKF_ZARU_REJECTED_INVALID_INPUT);
    TEST_EXPECT(memcmp(saved_quaternion, filter.q, sizeof(saved_quaternion)) == 0);
    TEST_EXPECT(memcmp(saved_bias, filter.gyro_bias_rad_s, sizeof(saved_bias)) == 0);
    TEST_EXPECT(memcmp(saved_covariance, filter.covariance, sizeof(saved_covariance)) == 0);

    const float invalid_rate[3] = {0.0f, NAN, 0.0f};
    TEST_EXPECT(attitude_mekf_update_zero_rate(
                    &filter, invalid_rate, measurement_covariance) ==
                ATTITUDE_MEKF_ZARU_REJECTED_INVALID_INPUT);
    TEST_EXPECT(memcmp(saved_quaternion, filter.q, sizeof(saved_quaternion)) == 0);
    TEST_EXPECT(memcmp(saved_covariance, filter.covariance, sizeof(saved_covariance)) == 0);
}

static void test_zaru_bias_limit(void)
{
    attitude_mekf_t filter;
    attitude_mekf_config_t config;
    const float large_rate[3] = {0.1f, 0.0f, 0.0f};
    const float covariance[3][3] = {
        {1.0e-8f, 0.0f, 0.0f},
        {0.0f, 1.0e-8f, 0.0f},
        {0.0f, 0.0f, 1.0e-8f},
    };

    attitude_mekf_default_config(&config);
    config.zaru_nis_gate = 1.0e6f;
    config.max_abs_bias_rad_s = 0.05f;
    TEST_EXPECT(attitude_mekf_init(&filter, &config));

    float saved_quaternion[4];
    float saved_bias[3];
    float saved_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    memcpy(saved_quaternion, filter.q, sizeof(saved_quaternion));
    memcpy(saved_bias, filter.gyro_bias_rad_s, sizeof(saved_bias));
    memcpy(saved_covariance, filter.covariance, sizeof(saved_covariance));

    TEST_EXPECT(attitude_mekf_update_zero_rate(&filter, large_rate, covariance) ==
                ATTITUDE_MEKF_ZARU_REJECTED_CORRECTION);
    TEST_EXPECT(memcmp(saved_quaternion, filter.q, sizeof(saved_quaternion)) == 0);
    TEST_EXPECT(memcmp(saved_bias, filter.gyro_bias_rad_s, sizeof(saved_bias)) == 0);
    TEST_EXPECT(memcmp(saved_covariance, filter.covariance, sizeof(saved_covariance)) == 0);
}

static void test_long_run_numerical_health(void)
{
    attitude_mekf_t filter;
    const float zero_delta[3] = {0.0f, 0.0f, 0.0f};
    const float static_force[3] = {0.0f, 0.0f, -TEST_GRAVITY};

    TEST_EXPECT(attitude_mekf_init(&filter, NULL));
    for (unsigned int sample = 0U; sample < 20000U; ++sample)
    {
        TEST_EXPECT(attitude_mekf_propagate_delta(&filter, zero_delta, TEST_DT_S));
        if ((sample % 4U) == 0U)
        {
            TEST_EXPECT(attitude_mekf_update_accel(&filter, static_force, 1.0f) ==
                        ATTITUDE_MEKF_ACCEL_ACCEPTED);
        }
    }
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
    TEST_EXPECT(filter.diagnostics.numeric_fault_count == 0U);
}

static void test_unobserved_rotation_only_inflates_attitude_covariance(void)
{
    attitude_mekf_t filter;
    TEST_EXPECT(attitude_mekf_init(&filter, NULL));

    float quaternion_before[4];
    float bias_before[3];
    float covariance_before[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    memcpy(quaternion_before, filter.q, sizeof(quaternion_before));
    memcpy(bias_before, filter.gyro_bias_rad_s, sizeof(bias_before));
    memcpy(covariance_before, filter.covariance, sizeof(covariance_before));

    TEST_EXPECT(attitude_mekf_mark_rotation_unobserved(&filter, 0.5f));
    TEST_EXPECT(memcmp(quaternion_before, filter.q, sizeof(quaternion_before)) == 0);
    TEST_EXPECT(memcmp(bias_before, filter.gyro_bias_rad_s, sizeof(bias_before)) == 0);
    for (size_t axis = 0U; axis < 3U; ++axis) {
        TEST_EXPECT(filter.covariance[axis][axis] >
                    covariance_before[axis][axis]);
        TEST_EXPECT(filter.covariance[axis + 3U][axis + 3U] ==
                    covariance_before[axis + 3U][axis + 3U]);
    }
    TEST_EXPECT(filter.diagnostics.unobserved_rotation_count == 1U);
    TEST_EXPECT(attitude_mekf_is_valid(&filter));

    TEST_EXPECT(!attitude_mekf_mark_rotation_unobserved(&filter, 0.0f));
    TEST_EXPECT(!attitude_mekf_mark_rotation_unobserved(&filter, NAN));
    TEST_EXPECT(filter.diagnostics.unobserved_rotation_count == 1U);
}

static void test_directional_unobserved_rotation_is_anisotropic(void)
{
    attitude_mekf_t filter;
    TEST_EXPECT(attitude_mekf_init(&filter, NULL));

    float quaternion_before[4];
    float bias_before[3];
    float covariance_before[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    memcpy(quaternion_before, filter.q, sizeof(quaternion_before));
    memcpy(bias_before, filter.gyro_bias_rad_s, sizeof(bias_before));
    memcpy(covariance_before, filter.covariance, sizeof(covariance_before));

    /* Baseline along +Y: the axis component gets the large std, the
     * perpendicular axes the small one. */
    const float axis[3] = {0.0f, 1.0f, 0.0f};
    TEST_EXPECT(attitude_mekf_mark_rotation_unobserved_directional(
        &filter, 0.5f, 0.01f, axis));
    TEST_EXPECT(memcmp(quaternion_before, filter.q,
                       sizeof(quaternion_before)) == 0);
    TEST_EXPECT(memcmp(bias_before, filter.gyro_bias_rad_s,
                       sizeof(bias_before)) == 0);
    const float axis_added = filter.covariance[1][1] -
                             covariance_before[1][1];
    const float perp_added_x = filter.covariance[0][0] -
                               covariance_before[0][0];
    const float perp_added_z = filter.covariance[2][2] -
                               covariance_before[2][2];
    TEST_EXPECT(fabsf(axis_added - 0.25f) < 0.01f);
    TEST_EXPECT(fabsf(perp_added_x - 1.0e-4f) < 1.0e-5f);
    TEST_EXPECT(fabsf(perp_added_z - 1.0e-4f) < 1.0e-5f);
    for (size_t axis_index = 0U; axis_index < 3U; ++axis_index) {
        TEST_EXPECT(filter.covariance[axis_index + 3U][axis_index + 3U] ==
                    covariance_before[axis_index + 3U][axis_index + 3U]);
    }
    TEST_EXPECT(filter.diagnostics.unobserved_rotation_count == 1U);
    TEST_EXPECT(attitude_mekf_is_valid(&filter));

    /* A zero perpendicular std with a positive axis std is legal (evidence
     * only along the axis); both zero is not, nor is a non-unit axis. */
    TEST_EXPECT(attitude_mekf_mark_rotation_unobserved_directional(
        &filter, 0.1f, 0.0f, axis));
    TEST_EXPECT(!attitude_mekf_mark_rotation_unobserved_directional(
        &filter, 0.0f, 0.0f, axis));
    const float skewed[3] = {0.5f, 0.5f, 0.0f};
    TEST_EXPECT(!attitude_mekf_mark_rotation_unobserved_directional(
        &filter, 0.1f, 0.1f, skewed));
    TEST_EXPECT(!attitude_mekf_mark_rotation_unobserved_directional(
        &filter, NAN, 0.1f, axis));

    /* Saturation at the attitude-variance cap reports success without
     * exceeding the cap, like the isotropic variant. */
    for (size_t repeat = 0U; repeat < 200U; ++repeat) {
        TEST_EXPECT(attitude_mekf_mark_rotation_unobserved_directional(
            &filter, 10.0f, 10.0f, axis));
    }
    const float cap = 0.5f * filter.config.covariance_ceiling;
    for (size_t axis_index = 0U; axis_index < 3U; ++axis_index)
        TEST_EXPECT(filter.covariance[axis_index][axis_index] <=
                    cap + 1.0e-6f);
    TEST_EXPECT(attitude_mekf_is_valid(&filter));
}

int main(void)
{
    test_initialization_and_validation();
    test_delta_angle_propagation();
    test_accel_seed_coordinate_convention();
    test_tilt_and_bias_convergence();
    test_gravity_update_at_arbitrary_attitude();
    test_off_norm_accel_updates_but_does_not_seed();
    test_accel_rejection_is_transactional();
    test_inflated_covariance_accel_correction_is_step_limited();
    test_yaw_unobservability_and_propagation_rejection();
    test_gravity_axis_bias_is_unobservable_at_tilt();
    test_noisy_tilt_preserves_gravity_gauge_and_observable_bias();
    test_zaru_three_axis_bias_update();
    test_bounded_zaru_clips_target_and_limits_global_slew();
    test_bounded_zaru_clips_about_existing_bias();
    test_zaru_rejection_is_transactional();
    test_zaru_bias_limit();
    test_unobserved_rotation_only_inflates_attitude_covariance();
    test_directional_unobserved_rotation_is_anisotropic();
    test_long_run_numerical_health();

    if (failure_count != 0U)
    {
        (void)fprintf(stderr, "attitude_mekf: %u test failure(s)\n", failure_count);
        return 1;
    }

    (void)puts("attitude_mekf: all tests passed");
    return 0;
}
