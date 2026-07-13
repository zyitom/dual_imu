#ifndef APP_H
#define APP_H

#include "imu_calibration.h"

#include <stdbool.h>

void app_init(void);
void app_process(void);

/* Override these weak hooks in the controller/parameter layer. */
bool app_load_imu_calibration(imu_source_t source,
                              imu_calibration_t *calibration);
bool app_imu_external_stationary_hint(void);

#endif
