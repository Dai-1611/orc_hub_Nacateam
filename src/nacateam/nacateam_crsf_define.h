#ifndef NACATEAM_CRSF_DEFINE_H
#define NACATEAM_CRSF_DEFINE_H

#include <Arduino.h>
#include "crsf/crsf.h"

#define ORC_BAUDRATE 420000
#define ORC_SERIAL Serial2
#define ORC_SERIAL_RX_PIN 8 // D5
#define ORC_SERIAL_TX_PIN 9 // D6
#define TUAT_VUONG_MODE 0
#define TU_VUONG_MODE 1
#define SAFETY_MODE -1

#define SPEED_MODE_0 50
#define SPEED_MODE_1 150
//--rc variable---//
#define SBUS_BUFFER_SIZE 25
extern uint8_t _rcs_buf[SBUS_BUFFER_SIZE];             // Định nghĩa trong naca_control.cpp
extern uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS]; // Định nghĩa trong naca_control.cpp
extern uint16_t _raw_rc_count;                         // Định nghĩa trong naca_control.cpp
extern int aileronsMapped;
extern int elevatorMapped;
extern int throttleMapped;
extern int rudderMapped;
extern int switchMapped1;
extern int switchMapped2;
extern int switchMapped3;
extern int switchMapped4;
//---rc variable---//

#endif // NACATEAM_CRSF_DEFINE_H
