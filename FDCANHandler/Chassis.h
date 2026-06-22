#ifndef CHASSIS_H
#define CHASSIS_H

#include "main.h"
#include "fdcan.h"
#include <stdint.h>
#include "FDCANHandler.h"
#include "Remote.h"

#define SWERVE_MODULE_NUM 4
#define SWERVE_TEST_ENABLE_SW 1
#define SWERVE_OPEN_LOOP_CURRENT_SCALE 4
#define SWERVE_OPEN_LOOP_MAX_CURRENT 3000
#define HALF_LENGTH 0.70710678f
typedef enum
{
    SWERVE_LF = 0,
    SWERVE_RF = 1,
    SWERVE_RR = 2,
    SWERVE_LR = 3,
} Swerve_Index_t;

typedef struct
{
    int16_t wheel_angle_raw;
    int16_t wheel_speed;
    int16_t wheel_current;
    int16_t wheel_temperature;

    float yaw_angle_deg;
    int16_t yaw_speed;
    int16_t yaw_current;
    int16_t yaw_temperature;
} Swerve_Feedback_t;

typedef struct
{
    int16_t wheel_current_cmd;
    int16_t yaw_current_cmd;
} Swerve_Command_t;

typedef struct
{
    Swerve_Feedback_t fb;
    Swerve_Command_t cmd;
    uint32_t feedback_count;
} Swerve_Module_t;

extern volatile Swerve_Module_t SwerveModule[SWERVE_MODULE_NUM];
extern uint8_t ChassisTxBuf[16];

void Chassis_Init(void);
void Chassis_TestControlFromRemote(void);
void debug(void);
void PID_Command(char *ch);
#endif
