// uln2003.h

#ifndef YDLIDAR_H_
#define YDLIDAR_H_

#include <stdint.h>
#include <stdbool.h>

#include "inc/v8stdint.h"

#define YDLIDAR_PWM 14
#define YDLIDAR_DATA 13
#define YDLIDAR_UART_SPEED 115200
#define FILEBUFFER_LEN (1024)

// dummy for uart init
#define YDLIDAR_TXD  19

struct ydlidarController {
    uint8_t pwm_val;
    void (*fileWriteFunction)(char* buffer);

    void (*init)(void);
    void (*start)(void);
    void (*stop)(void);
    // void init();
    void (*changePWM)(float pwm);
    // void step(int dir=1);
};

extern struct ydlidarController YdlidarController;
///////////////////////////////////////////////////// MINE ENDED ////////////////////////////////////////////////

#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x91
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x8000

#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81

#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT    8


#define LIDAR_CMD_RUN_POSITIVE             0x06
#define LIDAR_CMD_RUN_INVERSION            0x07
#define LIDAR_CMD_SET_AIMSPEED_ADDMIC      0x09
#define LIDAR_CMD_SET_AIMSPEED_DISMIC      0x0A
#define LIDAR_CMD_SET_AIMSPEED_ADD         0x0B
#define LIDAR_CMD_SET_AIMSPEED_DIS         0x0C
#define LIDAR_CMD_GET_AIMSPEED             0x0D
#define LIDAR_CMD_SET_SAMPLING_RATE        0xD0
#define LIDAR_CMD_GET_SAMPLING_RATE        0xD1

#define LIDAR_STATUS_OK                    0x0
#define LIDAR_STATUS_WARNING               0x1
#define LIDAR_STATUS_ERROR                 0x2

#define PackageSampleBytes 2
#define PackageSampleMaxLngth 0x80
#define Node_Default_Quality (10<<2)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA


typedef enum {
  CT_Normal = 0,
  CT_RingStart  = 1,
  CT_Tail,
} CT;

typedef struct {
  uint8_t    sync_quality;
  uint16_t   angle_q6_checkbit;
  uint16_t   distance_q2;
} node_info;

typedef struct {
  uint16_t  package_Head;
  uint8_t   package_CT;
  uint8_t   nowPackageNum;
  uint16_t  packageFirstSampleAngle;
  uint16_t  packageLastSampleAngle;
  uint16_t  checkSum;
  uint16_t  packageSampleDistance[PackageSampleMaxLngth];
} node_package;


typedef struct {
  uint8_t   model;
  uint16_t  firmware_version;
  uint8_t   hardware_version;
  uint8_t   serialnum[16];
} device_info ;

typedef struct  {
  uint8_t   status;
  uint16_t  error_code;
} device_health;

typedef struct {
  uint8_t rate;
} sampling_rate;

typedef struct {
  uint32_t frequency;
} scan_frequency;

typedef struct {
  uint8_t rotation;
} scan_rotation;

typedef struct {
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} cmd_packet ;

typedef struct {
  uint8_t  syncByte1;
  uint8_t  syncByte2;
  uint32_t size: 30;
  uint32_t subType: 2;
  uint8_t  type;
} lidar_ans_header;

typedef struct {
  uint8_t quality;
  float 	angle;
  float 	distance;
  bool    startBit;
} scanPoint;


scanPoint point;

#endif