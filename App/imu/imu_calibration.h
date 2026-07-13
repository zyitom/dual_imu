#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "imu_types.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CALIBRATION_TEMPERATURE_COEFFICIENT_COUNT (4U)

/*
 * Applied after the board's nominal axis rotation:
 *
 *   calibrated = correction_matrix * (nominal_body - bias(temperature))
 *
 * The matrices therefore combine residual sensor scale/non-orthogonality and
 * the measured residual sensor-to-common-body rotation. Bias coefficients are
 * c0..c3 in powers of (temperature-reference_temperature_c).
 */
typedef struct
{
    float accel_correction_matrix[3][3];
    float gyro_correction_matrix[3][3];
    float accel_bias_polynomial_mps2[3]
                                     [IMU_CALIBRATION_TEMPERATURE_COEFFICIENT_COUNT];
    float gyro_bias_polynomial_rad_s[3]
                                     [IMU_CALIBRATION_TEMPERATURE_COEFFICIENT_COUNT];
    float reference_temperature_c;
    float minimum_temperature_c;
    float maximum_temperature_c;
} imu_calibration_t;

void imu_calibration_initialize(void);
void imu_calibration_reset_all(void);
void imu_calibration_default(imu_calibration_t *calibration);
bool imu_calibration_validate(const imu_calibration_t *calibration);
bool imu_calibration_set(imu_source_t source,
                         const imu_calibration_t *calibration);
bool imu_calibration_get(imu_source_t source,
                         imu_calibration_t *calibration);
bool imu_calibration_has_custom(imu_source_t source);

bool imu_calibration_apply_accel(imu_source_t source,
                                 float temperature_c,
                                 float accel_mps2[3]);
bool imu_calibration_apply_gyro(imu_source_t source,
                                float temperature_c,
                                float gyro_rad_s[3]);

#ifdef __cplusplus
}
#endif

#endif
