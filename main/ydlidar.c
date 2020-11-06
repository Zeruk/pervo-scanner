#include "ydlidar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include <stdint.h>
#include "byteswap.h"

void initLIDAR();
void stopLIDAR();

void changePWM(float pwm);

// !!!! THIS IS LITTLE ENDIAN

static const char* TAG = "uart_YDLIDAR";
struct ydlidarController YdlidarController = {
    .pwm_val = 0,
    .init = initLIDAR,
    .stop = stopLIDAR,
    .changePWM = changePWM,
};
static const int RX_BUF_SIZE = 1024;

// double min_angle = angles::from_degrees(-180.f);
// double max_angle = angles::from_degrees(180.f);


// https://github.com/YDLIDAR/sdk/blob/master/include/ydlidar_protocol.h
// https://github.com/YDLIDAR/sdk/blob/master/src/CYdLidar.cpp#L210
struct my_node_info {
  uint8_t    sync_flag;  //sync flag
  uint16_t   sync_quality; //!信号质量
  uint16_t   angle_q6_checkbit; //!测距点角度
  uint16_t   distance_q2; //! 当前测距点距离
  uint64_t   stamp; //! 时间戳
  uint8_t    scan_frequence; //! 特定版本此值才有效,无效值是0
  uint8_t    debug_info[12];
  uint8_t    index;
};

scanPoint point;

struct my_node_info nodebuffer[100];

// wait response header
result_t waitResponseHeader(lidar_ans_header *header, uint32_t timeout) {
  int  recvPos = 0;
  uint32_t startTs = esp_timer_get_time()/1000;
  uint8_t  *headerBuffer = (uint8_t *)(header);
  uint32_t waitTime;

  uint8_t* currentbyte = (uint8_t*) malloc(1);
  while ((waitTime = esp_timer_get_time()/1000 - startTs) <= timeout) {
    // int currentbyte = _bined_serialdev->read();
    // uint8_t currentbyte;
    // uart_read_bytes(UART_NUM_1, currentbyte, 1, 1000 / portTICK_RATE_MS);

    if (uart_read_bytes(UART_NUM_1, currentbyte, 1, 1000 / portTICK_RATE_MS) < 0) {
      ESP_LOGE(TAG, "Empty uart");
      continue;
    }
    // ESP_LOGI(TAG, "currentbyte %02x", *currentbyte);

    switch (recvPos) {
        case 0:
        if (*currentbyte != LIDAR_ANS_SYNC_BYTE1) {
            continue;
        } 
        break;

        case 1:
        if (*currentbyte != LIDAR_ANS_SYNC_BYTE2) {
            recvPos = 0;
            continue;
        }

        break;
    }
    headerBuffer[recvPos++] = *currentbyte;
    if (recvPos == sizeof(lidar_ans_header)) {
        free(currentbyte);
      return RESULT_OK;
    }
  }
  free(currentbyte);
  return RESULT_TIMEOUT;
};

result_t getDeviceInfo(device_info *info, uint32_t timeout) {
    result_t ans;
    uint8_t  recvPos = 0;
    uint32_t currentTs = esp_timer_get_time()/1000;
    uint32_t remainingtime;
    uint8_t *infobuf = (uint8_t *)&info;
    lidar_ans_header* response_header=malloc(sizeof(lidar_ans_header));

    ESP_LOGI(TAG, "waitResponseHeader");
    if ((ans = waitResponseHeader(response_header, timeout)) != RESULT_OK) {
        return ans;
    }

    ESP_LOGI(TAG, "afterwaitResponseHeader type:%d size:%d sizeof:%d", response_header->type, response_header->size, sizeof(lidar_ans_header));
// I (832) uart_YDLIDAR: afterwaitResponseHeader 46 67371008 12

    if (response_header->type != LIDAR_ANS_TYPE_DEVINFO) {
        free(response_header);
        return RESULT_FAIL;
    }

    if (response_header->size < sizeof(lidar_ans_header)) {
        free(response_header);
        return RESULT_FAIL;
    }

    uint8_t* currentbyte = (uint8_t*) malloc(1);
    while ((remainingtime = esp_timer_get_time()/1000 - currentTs) <= timeout) {
        if (uart_read_bytes(UART_NUM_1, currentbyte, 1, 1000 / portTICK_RATE_MS) < 0) {
            continue;
        }

        infobuf[recvPos++] = *currentbyte;

        if (recvPos == sizeof(device_info)) {
            free(currentbyte);
            return RESULT_OK;
        }
    }

  return RESULT_TIMEOUT;
}


result_t startScan(bool force, uint32_t timeout) {
  result_t ans;

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
        ESP_LOGI(TAG, "Response_header.type = %d", response_header.type);
        return RESULT_FAIL;
    }

    if (response_header.size < sizeof(node_info)) {
      return RESULT_FAIL;
    }
  return RESULT_OK;
}

// wait scan data
result_t waitScanDot(uint32_t timeout) {
  int recvPos = 0;
  uint32_t startTs = esp_timer_get_time()/1000;
  uint32_t waitTime;
  uint8_t nowPackageNum;
  node_info node;
  static node_package package;
  static uint16_t package_Sample_Index = 0;
  static float IntervalSampleAngle = 0;
  static float IntervalSampleAngle_LastPackage = 0;
  static uint16_t FirstSampleAngle = 0;
  static uint16_t LastSampleAngle = 0;
  static uint16_t CheckSum = 0;

  static uint16_t CheckSumCal = 0;
  static uint16_t SampleNumlAndCTCal = 0;
  static uint16_t LastSampleAngleCal = 0;
  static bool CheckSumResult = true;
  static uint16_t Valu8Tou16 = 0;

  uint8_t *packageBuffer = (uint8_t *)&package.package_Head;
  uint8_t  package_Sample_Num = 0;
  int32_t AngleCorrectForDistance;

  int  package_recvPos = 0;

  if (package_Sample_Index == 0) {
    recvPos = 0;

    while ((waitTime = (esp_timer_get_time()/1000 - startTs)) <= timeout) {
      uint8_t* currentByte = (uint8_t*) malloc(1);
      if (uart_read_bytes(UART_NUM_1, currentByte, 1, 1000 / portTICK_RATE_MS) < 0) {
        vTaskDelay(5 / portTICK_PERIOD_MS);
        continue;
      }

      switch (recvPos) {
      case 0:
        if (*currentByte != (PH & 0xFF)) {
          continue;
        }

        break;

      case 1:
        CheckSumCal = PH;

        if (*currentByte != (PH >> 8)) {
          recvPos = 0;
          continue;
        }

        break;

      case 2:
        SampleNumlAndCTCal = *currentByte;

        if (((*currentByte&0x01) != CT_Normal) && ((*currentByte & 0x01) != CT_RingStart)) {
          recvPos = 0;
          continue;
        }

        break;

      case 3:
        SampleNumlAndCTCal += (*currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        package_Sample_Num = *currentByte;
        break;

      case 4:
        if (*currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
          FirstSampleAngle = *currentByte;
        } else {
          recvPos = 0;
          continue;
        }

        break;

      case 5:
        FirstSampleAngle += (*currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        CheckSumCal ^= FirstSampleAngle;
        FirstSampleAngle = FirstSampleAngle >> 1;
        break;

      case 6:
        if (*currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
          LastSampleAngle = *currentByte;
        } else {
          recvPos = 0;
          continue;
        }

        break;

      case 7:
        LastSampleAngle += (*currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        LastSampleAngleCal = LastSampleAngle;
        LastSampleAngle = LastSampleAngle >> 1;

        if (package_Sample_Num == 1) {
          IntervalSampleAngle = 0;
        } else {
          if (LastSampleAngle < FirstSampleAngle) {
            if ((FirstSampleAngle > 17280) && (LastSampleAngle < 5760)) {
              IntervalSampleAngle = ((float)(23040 + LastSampleAngle - FirstSampleAngle)) /
                                    (package_Sample_Num - 1);
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            } else {
              IntervalSampleAngle = IntervalSampleAngle_LastPackage;
            }
          } else {
            IntervalSampleAngle = ((float)(LastSampleAngle - FirstSampleAngle)) / (package_Sample_Num - 1);
            IntervalSampleAngle_LastPackage = IntervalSampleAngle;
          }
        }

        break;

      case 8:
        CheckSum = *currentByte;
        break;

      case 9:
        CheckSum += (*currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        break;
      }

      packageBuffer[recvPos++] = *currentByte;

      if (recvPos  == PackagePaidBytes) {
        package_recvPos = recvPos;
        break;

      }
    }

    if (PackagePaidBytes == recvPos) {
    //   startTs = millis();
      recvPos = 0;
      int package_sample_sum = package_Sample_Num << 1;

      while (1) { //(waitTime = millis() - startTs) <= timeout) {
        uint8_t* currentByte = (uint8_t*) malloc(1);
        if (uart_read_bytes(UART_NUM_1, currentByte, 1, 1000 / portTICK_RATE_MS) < 0) {
          continue;
        }

        if ((recvPos & 1) == 1) {
          Valu8Tou16 += (*currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          CheckSumCal ^= Valu8Tou16;
        } else {
          Valu8Tou16 = *currentByte;
        }

        packageBuffer[package_recvPos + recvPos] = *currentByte;
        recvPos++;

        if (package_sample_sum == recvPos) {
          package_recvPos += recvPos;
          break;
        }
      }

      if (package_sample_sum != recvPos) {
        return RESULT_FAIL;
      }
    } else {
      return RESULT_FAIL;
    }

    CheckSumCal ^= SampleNumlAndCTCal;
    CheckSumCal ^= LastSampleAngleCal;

    if (CheckSumCal != CheckSum) {
      CheckSumResult = false;
    } else {
      CheckSumResult = true;
    }

  }

  uint8_t package_CT;
  package_CT = package.package_CT;

  if ((package_CT&0x01) == CT_Normal) {
    node.sync_quality = Node_Default_Quality + Node_NotSync;
  } else {
    node.sync_quality = Node_Default_Quality + Node_Sync;
  }

  if (CheckSumResult == true) {
    node.distance_q2 = package.packageSampleDistance[package_Sample_Index];

    if (node.distance_q2 / 4 != 0) {
      AngleCorrectForDistance = (int32_t)((atan(((21.8 * (155.3 - (node.distance_q2 * 0.25f))) /
                                           155.3) / (node.distance_q2 * 0.25f))) * 3666.93);
    } else {
      AngleCorrectForDistance = 0;
    }

    float sampleAngle = IntervalSampleAngle * package_Sample_Index;

    if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0) {
      node.angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance +
                                            23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
    } else {
      if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040) {
        node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance -
                                              23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      } else {
        node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance)) <<
                                  LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  } else {
    node.sync_quality = Node_Default_Quality + Node_NotSync;
    node.angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    node.distance_q2 = 0;
    package_Sample_Index = 0;
    return RESULT_FAIL;
  }

  point.distance = node.distance_q2 * 0.25f;
  point.angle = (node.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
  point.quality = (node.sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
  point.startBit = (node.sync_quality & LIDAR_RESP_MEASUREMENT_SYNCBIT);

  package_Sample_Index++;
  nowPackageNum = package.nowPackageNum;

  if (package_Sample_Index >= nowPackageNum) {
    package_Sample_Index = 0;
  }

  return RESULT_OK;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

static void rx_task(void *arg)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    char* fileBuffer = (char*) malloc(FILEBUFFER_LEN);
    char* strBuffer = (char*) malloc(100);
    float range = 0.0;
    uint8_t intensity = 0.0;
    float angle = 0.0;
    // device_info* deviceinfo = (device_info*) malloc(1);
    // result_t getInfoResult;

    /// startScan
    while (1) {
      uart_flush(UART_NUM_1);
      /// Get device info
      // getInfoResult = getDeviceInfo(deviceinfo, 5000);
      // if(getInfoResult != RESULT_OK) {
      //   ESP_LOGE(TAG, "Didn't get device info");
      //   if(getInfoResult == RESULT_TIMEOUT) ESP_LOGE(TAG, "Because of timeout");
      //   vTaskDelay(10 / portTICK_PERIOD_MS);
      //   continue;
      // }
      
      // uint16_t maxv = (uint16_t)(deviceinfo->firmware_version>>8);
      // uint16_t midv = (uint16_t)(deviceinfo->firmware_version&0xff)/10;
      // uint16_t minv = (uint16_t)(deviceinfo->firmware_version&0xff)%10;
      // if(midv==0){
      //     midv = minv;
      //     minv = 0;
      // }
      
      // ESP_LOGI(TAG, "!!!  device_info: model %d, v%d.%d.%d, hv:%d", deviceinfo->model,maxv, midv, minv, deviceinfo->hardware_version);
      // // Strange version
      // free(deviceinfo);

      if(startScan(false, 5000) != RESULT_OK) {
        ESP_LOGE(TAG, "Didn't started scan");
        vTaskDelay(10 / portTICK_PERIOD_MS);
        continue;
      }

      while(1) {
          vTaskDelay(1 / portTICK_PERIOD_MS);
          if (waitScanDot(1000) == RESULT_OK) {
              range     = point.distance; //distance value in mm unit
              angle     = point.angle; //anglue value in degree
              intensity = point.quality; //quality of the current measurement
            // bool  startBit = point.startBit;
              // ESP_LOGI(TAG, "Read current angle: %f, distance: %f, quality: %d", angle, distance, quality);
              if(strlen(fileBuffer) > FILEBUFFER_LEN - 40) {
                // printf("================= Write to SDCard: =============================\n%s", fileBuffer);
                ESP_LOGI(TAG, "Write fileBuffer to SDCard");
                YdlidarController.fileWriteFunction(fileBuffer);
                fileBuffer[0] = '\0';
              }
              sprintf(strBuffer, "%f %f %d\n", angle, range, intensity);
              strcat(fileBuffer, strBuffer);
              vTaskDelay(10 / portTICK_PERIOD_MS); // for background processes
        }else{
          ESP_LOGI(TAG, "YDLIDAR get Scandata fialed!!");
        }
      }
    }
};

void configurePWM() {
    //// from mcpwm_servo_control

    //1. mcpwm gpio initialization
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, YDLIDAR_PWM);

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 10000;    //frequency = 10kHz
    pwm_config.cmpr_a = 50;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 50;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    // 20ms - full period
    // 10ms - half (50%)
    // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 100);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 100);
};

void configureUART() {
    //// from uart_async_rxtxtasks

    const uart_config_t uart_config = {
        .baud_rate = YDLIDAR_UART_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, YDLIDAR_TXD, YDLIDAR_DATA, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
};


TaskHandle_t ydlidarRXTaskHandle = NULL;

void initLIDAR() { 
    ///////// CONFIGURE PWM /////////
    configurePWM();

    ///////// CONFIGURE YDLIDAR /////////
    point.distance = 0;
    point.angle = 0;
    point.quality = 0;

    ////////// CONFIGURE UART ///////////
    configureUART();
    // Listen on UART
    xTaskCreate(rx_task, "ydlidar_rx_task", 1024*8, NULL, configMAX_PRIORITIES, &ydlidarRXTaskHandle);
}
void stopLIDAR() {
  vTaskDelete(ydlidarRXTaskHandle);
}
void changePWM(float pwm) {
    YdlidarController.pwm_val=pwm;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm);
}


// struct ydlidarController YdlidarController = {
//     .pwm_val = 0,
//     .init = init,
//     .changePWM = changePWM,
// };
