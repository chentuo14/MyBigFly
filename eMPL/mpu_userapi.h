
#ifndef __MPU_USERAPI_H_
#define __MPU_USERAPI_H_

#include "stm32f10x.h"
#include <stdio.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "eMPL_outputs.h"
#include "log.h"
#include "packet.h"
#include "hal_outputs.h"
#include "message_layer.h"
#include "data_builder.h"
#include "mpl.h"
#include "accel_auto_cal.h"
#include "compass_vec_cal.h"
#include "fast_no_motion.h"
#include "fusion_9axis.h"
#include "gyro_tc.h"
#include "heading_from_gyro.h"
#include "mag_disturb.h"
#include "motion_no_motion.h"
#include "no_gyro_fusion.h"
#include "quaternion_supervisor.h"
#include "inv_math.h"
#include "ml_math_func.h"
#include  "fusion_9axis.h"

#include "i2c.h"

void handle_input(void);
int user_main(void);
int mpu9250_initialize(void);
uint8_t MPU6050_getDeviceID(void);
u8 mpu_mpl_get_data(float *pitch,float *roll,float *yaw, vs16 *gyro_x, vs16 *gyro_y);


#endif
