#include <Arduino.h>
#include "naca_control/naca_control.h"

void setup()
{
    naca_control_init();
}
void loop()
{
    // Nút D7/D8 chạy chế độ tự động (chặn đến khi xong), còn lại là điều khiển tay
    if (!naca_auto_handle_loop())
    {
        naca_manual_handle_loop();
    }
    //lift_motor(LIFT_HOLD_SPEED); // giữ để cơ cấu nâng không rơi
    //servo_mid_gripper.spin(0);
   // servo_mid_gripper.spin(single_mid_left_servo_pickup_ball_close_ms);
    //servo_mid_gripper.spin(single_mid_right_servo_pickup_ball_close_ms);
    //servo_left_gripper.spin(single_left_servo_pickup_ball_close_ms);
 //   servo_right_gripper.spin(single_right_servo_pickup_ball_open_ms);
}
