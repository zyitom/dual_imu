#ifndef IMU_GEOMETRY_H
#define IMU_GEOMETRY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DM-FC01 component centers from the vendor STEP model:
 *   BMI088 (U11): x=-7.1 mm, y=2.401 mm
 *   ICM45686 (U13 package model): x=0 mm, y=2.401 mm
 *
 * Both sensors are ROTATION_PITCH_180. In the resulting FRD body frame the
 * center-to-center vector from ICM45686 to BMI088 is approximately
 * [0, -7.1 mm, 0]. The common virtual-IMU origin is their midpoint.
 * Package geometry does not locate the internal MEMS die along Z, so the
 * initial Z offset is intentionally zero and can be replaced by calibration.
 */
#define IMU_DM_FC01_CENTER_DISTANCE_M (0.0071f)

/*
 * Optional mechanical-CAD vector from the desired rigid-body reference point
 * to the midpoint of the two IMUs, expressed in FRD body axes. Leave all three
 * at zero to publish specific force at the dual-IMU midpoint. For a gimbal,
 * the useful reference is the yaw/pitch axis intersection, not the PCB center.
 */
#ifndef IMU_REFERENCE_TO_MIDPOINT_X_M
#define IMU_REFERENCE_TO_MIDPOINT_X_M (0.0f)
#endif
#ifndef IMU_REFERENCE_TO_MIDPOINT_Y_M
#define IMU_REFERENCE_TO_MIDPOINT_Y_M (0.0f)
#endif
#ifndef IMU_REFERENCE_TO_MIDPOINT_Z_M
#define IMU_REFERENCE_TO_MIDPOINT_Z_M (0.0f)
#endif

extern const float imu_dm_fc01_midpoint_to_bmi_m[3];
extern const float imu_dm_fc01_midpoint_to_icm_m[3];
extern const float imu_dm_fc01_reference_to_bmi_m[3];
extern const float imu_dm_fc01_reference_to_icm_m[3];

typedef struct
{
    float previous_gyro_rad_s[3];
    float angular_accel_rad_s2[3];
    float cutoff_hz;
    float limit_rad_s2;
    bool initialized;
} imu_angular_accel_estimator_t;

void imu_angular_accel_estimator_init(imu_angular_accel_estimator_t *estimator,
                                      float cutoff_hz,
                                      float limit_rad_s2);

bool imu_angular_accel_estimator_update(imu_angular_accel_estimator_t *estimator,
                                       const float gyro_rad_s[3],
                                       float dt_s,
                                       float angular_accel_rad_s2[3]);

/*
 * Translate specific force from a sensor point to a rigid-body reference:
 *
 *   f_reference = f_sensor - alpha x r - omega x (omega x r)
 *
 * where r points from the reference to the sensor, expressed in body axes.
 * Input and output may refer to the same array.
 */
bool imu_translate_specific_force(const float sensor_specific_force_mps2[3],
                                  const float angular_rate_rad_s[3],
                                  const float angular_accel_rad_s2[3],
                                  const float reference_to_sensor_m[3],
                                  float reference_specific_force_mps2[3]);

/*
 * Window-mean form of the same translation. angular_rate_second_moment is
 * E[omega*omega'] over the accel averaging window, so zero-mean angular
 * vibration still contributes its real mean centripetal acceleration.
 */
bool imu_translate_mean_specific_force(
    const float sensor_specific_force_mps2[3],
    const float *angular_rate_second_moment_rad2_s2,
    const float mean_angular_accel_rad_s2[3],
    const float reference_to_sensor_m[3],
    float reference_specific_force_mps2[3]);

#ifdef __cplusplus
}
#endif

#endif
