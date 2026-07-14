#ifndef APP_H
#define APP_H

#include "imu_calibration.h"

#include <stdbool.h>

void app_init(void);
void app_process(void);

/* Override this weak hook in the parameter-storage layer. */
bool app_load_imu_calibration(imu_source_t source,
                              imu_calibration_t *calibration);

#endif
