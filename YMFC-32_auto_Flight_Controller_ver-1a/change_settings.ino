///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the settings that can be set via the transmitter are handled.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Changing the settings is explained in this video:
//https://youtu.be/ys-YpOaA2ME
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void change_settings(void) {
  red_led(HIGH);
  green_led(LOW);
  adjustable_setting_1 = variable_1_to_adjust;
  adjustable_setting_2 = variable_2_to_adjust;
  adjustable_setting_3 = variable_3_to_adjust;

  for (error = 0; error < 150; error ++) {
    delay(20);
    send_telemetry_data();
  }
  error = 0;

  while (channel_6 >= 1900) {
    delayMicroseconds(3700);
    send_telemetry_data();
    if (channel_1 > 1550)adjustable_setting_1 += (float)(channel_1 - 1550) * 0.000001;
    if (channel_1 < 1450)adjustable_setting_1 -= (float)(1450 - channel_1) * 0.000001;
    if (adjustable_setting_1 < 0)adjustable_setting_1 = 0;
    variable_1_to_adjust = adjustable_setting_1;
    
    if (channel_2 > 1550)adjustable_setting_2 += (float)(channel_2 - 1550) * 0.000001;
    if (channel_2 < 1450)adjustable_setting_2 -= (float)(1450 - channel_2) * 0.000001;
    if (adjustable_setting_2 < 0)adjustable_setting_2 = 0;
    variable_2_to_adjust = adjustable_setting_2;

    if (channel_4 > 1550)adjustable_setting_3 += (float)(channel_4 - 1550) * 0.000001;
    if (channel_4 < 1450)adjustable_setting_3 -= (float)(1450 - channel_4) * 0.000001;
    if (adjustable_setting_3 < 0)adjustable_setting_3 = 0;
    variable_3_to_adjust = adjustable_setting_3;
  }
  loop_timer = micros();                                                           //Set the timer for the next loop.
  red_led(LOW);
}
