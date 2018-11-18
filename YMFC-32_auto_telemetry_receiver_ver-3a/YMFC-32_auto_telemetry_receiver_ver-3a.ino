///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
#include <LiquidCrystal.h>
#include <EEPROM.h>


uint8_t receive_buffer[50], receive_buffer_counter, receive_byte_previous, receive_start_detect;
uint8_t check_byte, temp_byte;
uint8_t error, alarm_sound, flight_mode;
uint8_t telemetry_lost;
uint8_t alarm_silence;
uint8_t start, flight_timer_start;
uint8_t hours,minutes, seconds;
uint8_t heading_lock;
uint8_t number_used_sats;
uint8_t fix_type, max_speed_from_eeprom, speed_kmph, max_speed, speed_loop_counter;
uint16_t speed_buffer[5];

int8_t page, previous_page;

uint32_t last_receive, next_sound, flight_timer, flight_timer_previous, flight_timer_from_start, flight_time_from_eeprom;
uint32_t hours_flight_time, minutes_flight_time, seconds_flight_time;
int32_t l_lat_gps, l_lon_gps, l_lat_gps_previous, l_lon_gps_previous;
float lat_distance, lon_distance;

int16_t temperature, button_push, button_store, roll_angle, pitch_angle;
int16_t altitude_meters, max_altitude_meters, max_altitude_from_eeprom;
uint16_t key_press_timer;
int16_t takeoff_throttle;
uint16_t actual_compass_heading;

float battery_voltage, adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;

byte led;

//SoftwareSerial TelemetrySerial(11, 12); // RX, TX

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup() {
  //Let's setup a interrupt that will check the key inputs on the analog input A0.
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS22|1 << CS21|1 << CS20);                                //Set the CS20, CS21 and CS22 bit in the TCCRB register to set the prescaler to 1024
  OCR2A = 249;                                                              //The compare register is set to 249 => 16ms / (1s / (16.000.000MHz / 1024)) - 1 = 249
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode


  Serial.begin(9600);                                                       //Set the serial output to 9600bps.
  pinMode(2, OUTPUT);                                                       //Set input 2 as output for the buzzer.

  lcd.begin(16, 2);                                                         // set up the LCD's number of columns and rows.
  // Print a welcome message to the LCD.
  lcd.setCursor(4, 0);
  lcd.print("YMFC-32");
  lcd.setCursor(3, 1);
  lcd.print("telemetry");
  //Create a signal to indicate the start of the program.
  digitalWrite(2, HIGH);
  delay(10);
  digitalWrite(2, LOW);
  delay(50);
  digitalWrite(2, HIGH);
  delay(10);
  digitalWrite(2, LOW);
  delay(2500);

  button_store = -1;
  lcd.clear();
  telemetry_lost = 2;
  flight_time_from_eeprom = (uint32_t)EEPROM.read(0x00)<< 24 | (uint32_t)EEPROM.read(0x01)<< 16 | (uint32_t)EEPROM.read(0x02)<< 8 | (uint32_t)EEPROM.read(0x03);
  max_altitude_from_eeprom = EEPROM.read(0x04)<< 8 | EEPROM.read(0x05);
  max_speed_from_eeprom = (uint32_t)EEPROM.read(0x06);
  max_speed = max_speed_from_eeprom;
}

void loop() {
  if(key_press_timer > 0){
    key_press_timer = 0;
    button_store = -1;
  }

  if(button_store != -1){
    while(analogRead(0) < 1000){
      delay(10);
      if(key_press_timer < 200)key_press_timer ++;
      if(key_press_timer == 200){
        digitalWrite(2, HIGH);
        delay(10);
        digitalWrite(2, LOW);
        delay(50);
        digitalWrite(2, HIGH);
        delay(10);
        digitalWrite(2, LOW);
        delay(500);

      }
    }
    if(key_press_timer < 200){
      digitalWrite(2, HIGH);
      delay(10);
      digitalWrite(2, LOW);
    }
  }

  if((telemetry_lost == 1 || alarm_sound == 1) && button_store != -1){
    if(alarm_sound == 1)alarm_sound = 2;
    if(telemetry_lost == 1)telemetry_lost = 2;
    page = 0;
    lcd.clear();
    button_store = -1;
  }

  if(button_store != -1 && key_press_timer < 200){
    if(button_store < 300 && button_store > 200)page--; //Down
    if(button_store < 150 && button_store > 50)page++; //Up
    if(button_store < 700 && button_store > 600)page=0; //Select
    if(page < 0)page = 0;
    if(page > 6)page = 7;
    button_store = -1;
  }

  if(start > 1 && flight_timer_start == 0){
    flight_time_from_eeprom = (uint32_t)EEPROM.read(0x00)<< 24 | (uint32_t)EEPROM.read(0x01)<< 16 | (uint32_t)EEPROM.read(0x02)<< 8 | (uint32_t)EEPROM.read(0x03);
    flight_timer_from_start = millis();
    flight_timer = millis() - flight_timer_previous;
    flight_timer_start = 1;
  }

  if(start == 0 && flight_timer_start == 1){
    if(max_altitude_meters > max_altitude_from_eeprom){
      max_altitude_from_eeprom = max_altitude_meters;
      EEPROM.write(0x04, max_altitude_from_eeprom >> 8);
      EEPROM.write(0x05, max_altitude_from_eeprom);
    }

    if(max_speed > max_speed_from_eeprom){
      max_speed_from_eeprom = max_speed;
      EEPROM.write(0x06, max_speed_from_eeprom);
    }

    flight_time_from_eeprom += (millis() - flight_timer_from_start)/1000;
    EEPROM.write(0x00, flight_time_from_eeprom >> 24);
    EEPROM.write(0x01, flight_time_from_eeprom >> 16);
    EEPROM.write(0x02, flight_time_from_eeprom >> 8);
    EEPROM.write(0x03, flight_time_from_eeprom);

    flight_timer_previous = millis() - flight_timer;
    flight_timer_start = 0;
  }

  while(Serial.available()){                                                //If there are bytes available.      
    receive_buffer[receive_buffer_counter] = Serial.read();                 //Load them in the received_buffer array.
    //Search for the start signature in the received data stream.
    if(receive_byte_previous == 'J' && receive_buffer[receive_buffer_counter] == 'B'){
      receive_buffer_counter = 0;                                           //Reset the receive_buffer_counter counter if the start signature if found.
      receive_start_detect ++;                                              //Increment the receive_start_detect to check for a full data stream reception.
      if(receive_start_detect >= 2)get_data();                              //If there are two start signatures detected there could be a complete data set available.
    }
    else{                                                                   //If there is no start signature detected.
      receive_byte_previous = receive_buffer[receive_buffer_counter];       //Safe the current received byte for the next loop.
      receive_buffer_counter ++;                                            //Increment the receive_buffer_counter variable.
      if(receive_buffer_counter > 48)receive_buffer_counter = 0;            //Reset the receive_buffer_counter variable when it becomes larger than 38.
    }
  }

  if(start > 1){
    minutes = (millis() - flight_timer)/60000;
    seconds = ((millis() - flight_timer)-minutes*60000)/1000;
  }

  if(page != previous_page){
    previous_page = page;
    lcd.clear();
  }

  if(page == 0){
    if(flight_mode <= 3){
      lcd.setCursor(0, 0);
      lcd.print("M");
      lcd.print(flight_mode);
    }
    else {
      lcd.setCursor(0, 0);
      lcd.print("R");
      lcd.print(flight_mode - 4);
    }

    lcd.setCursor(5, 0);
    if(battery_voltage < 10)lcd.print("0");
    lcd.print(battery_voltage,1);
    lcd.print("V ");

    lcd.setCursor(11, 0);
    if(altitude_meters < 0)lcd.print("-");
    else lcd.print("+");
    if(altitude_meters < 100)lcd.print("0");
    if(altitude_meters < 10)lcd.print("0");
    lcd.print(abs(altitude_meters));
    lcd.print("m");

    lcd.setCursor(2, 0);
    lcd.print("E");
    lcd.print(error);

    lcd.setCursor(0, 1);
    if(fix_type == 3)lcd.print("S");
    else lcd.print("s");
    if(number_used_sats < 10)lcd.print("0");
    lcd.print(number_used_sats);

    lcd.setCursor(5, 1);
    if(minutes < 10)lcd.print("0");
    lcd.print(minutes);
    lcd.print(":");
    if(seconds < 10)lcd.print("0");
    lcd.print(seconds);

    lcd.setCursor(11, 1);
    lcd.print("H");
    if(actual_compass_heading < 100)lcd.print("0");
    if(actual_compass_heading < 10)lcd.print("0");
    lcd.print(actual_compass_heading);
    if(heading_lock)lcd.print("L");
    else lcd.print((char)223);
  }

  if(page == 1){
    if(key_press_timer == 200){
      button_store = -1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reset max spd?");
      lcd.setCursor(0, 1);
      lcd.print("Select = yes");
      while(button_store == -1)delay(10);
      if(button_store < 700 && button_store > 600){
        while(analogRead(0) < 1000)delay(10);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Max spd is reset");
        EEPROM.write(0x06, 0x00);
        max_speed_from_eeprom = EEPROM.read(0x06);
        max_speed = max_speed_from_eeprom;
        delay(2000);
      }
      lcd.clear();
    }
    else{
      lcd.setCursor(0, 0);
      lcd.print("Speed     Max");
      lcd.setCursor(0, 1);
      if(speed_kmph < 100)lcd.print("0");
      if(speed_kmph < 10)lcd.print("0");
      lcd.print(speed_kmph);
      lcd.print("kph    ");
      if(max_speed < 100)lcd.print("0");
      if(max_speed < 10)lcd.print("0");
      lcd.print(max_speed);
      lcd.print("kph");
    }
  }

  if(page == 2){
    lcd.setCursor(0, 0);
    lcd.print("Lat:");
    lcd.print(l_lat_gps);
    lcd.setCursor(0, 1);
    lcd.print("Lon:");
    lcd.print(l_lon_gps);
  }

  if(page == 3){
    if(key_press_timer == 200){
      button_store = -1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reset max alt?");
      lcd.setCursor(0, 1);
      lcd.print("Select = yes");
      while(button_store == -1)delay(10);
      if(button_store < 700 && button_store > 600){
        while(analogRead(0) < 1000)delay(10);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Max alt is reset");
        EEPROM.write(0x04, 0x00);
        EEPROM.write(0x05, 0x00);
        max_altitude_from_eeprom = EEPROM.read(0x04)<< 8 | EEPROM.read(0x05);
        delay(2000);
      }
      lcd.clear();
    }
    else{
      lcd.setCursor(0, 0);
      lcd.print("Max altitude:");
      lcd.setCursor(0, 1);
      if(max_altitude_meters < 100)lcd.print("0");
      if(max_altitude_meters < 10)lcd.print("0");
      lcd.print(max_altitude_meters);
      lcd.print("m     mem");
      if(max_altitude_from_eeprom < 100)lcd.print("0");
      if(max_altitude_from_eeprom < 10)lcd.print("0");
      lcd.print(max_altitude_from_eeprom);
      lcd.print("m");
    }
  }

  if(page == 4){
    lcd.setCursor(0, 0);
    lcd.print("roll: ");
    if(roll_angle >= 0)lcd.print("+");
    else lcd.print("-");
    if(roll_angle < 10 && roll_angle > -10)lcd.print("0");
    lcd.print(abs(roll_angle));
    lcd.setCursor(0, 1);
    lcd.print("pitch:");
    if(pitch_angle >= 0)lcd.print("+");
    else lcd.print("-");
    if(pitch_angle < 10 && pitch_angle > -10)lcd.print("0");
    lcd.print(abs(pitch_angle));
  }

  if(page == 5){
    if(key_press_timer == 200){
      button_store = -1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reset timer?");
      lcd.setCursor(0, 1);
      lcd.print("Select = yes");
      while(button_store == -1)delay(10);
      if(button_store < 700 && button_store > 600){
        while(analogRead(0) < 1000)delay(10);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Timer is reset");
        EEPROM.write(0x00, 0x00);
        EEPROM.write(0x01, 0x00);
        EEPROM.write(0x02, 0x00);
        EEPROM.write(0x03, 0x00);
        flight_time_from_eeprom = (uint32_t)EEPROM.read(0x00)<< 24 | (uint32_t)EEPROM.read(0x01)<< 16 | (uint32_t)EEPROM.read(0x02)<< 8 | EEPROM.read(0x03);
        delay(2000);
      }
      //telemetry_lost = 2;
      lcd.clear();
    }
    else{
      lcd.setCursor(0, 0);
      lcd.print("Tot flight time");
      lcd.setCursor(0, 1);
      hours_flight_time = flight_time_from_eeprom/3600;
      minutes_flight_time = (flight_time_from_eeprom - (hours_flight_time*3600))/60;
      seconds_flight_time = flight_time_from_eeprom - (hours_flight_time*3600) - (minutes_flight_time*60);
      if(hours_flight_time < 10)lcd.print("0");
      lcd.print(hours_flight_time);
      lcd.print(":");
      if(minutes_flight_time < 10)lcd.print("0");
      lcd.print(minutes_flight_time);
      lcd.print(":");
      if(seconds_flight_time < 10)lcd.print("0");
      lcd.print(seconds_flight_time);
    }
  }

  if(page == 6){
    lcd.setCursor(0, 0);
    lcd.print("1:");
    if(adjustable_setting_1 < 10)lcd.print("0");
    lcd.print(adjustable_setting_1);
    lcd.setCursor(8, 0);
    lcd.print("3:");
    if(adjustable_setting_3 < 10)lcd.print("0");
    lcd.print(adjustable_setting_3);
    lcd.setCursor(0, 1);
    lcd.print("2:");
    if(adjustable_setting_2 < 10)lcd.print("0");
    lcd.print(adjustable_setting_2);

  }

  if(page == 7){
    lcd.setCursor(0, 0);
    lcd.print("Take-off thr:");
    lcd.setCursor(0, 1);
    lcd.print(takeoff_throttle);      
  }

  if(page == 100){
    lcd.setCursor(0, 0);
    lcd.print(" Lost telemetry");
    lcd.setCursor(0, 1);
    lcd.print("   connection!"); 
    delay(1000);    
  }
  if(page > 100){
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.print(error);
    lcd.setCursor(0, 1);
    if(error == 1)lcd.print("Battery LOW!");
    if(error == 5)lcd.print("Loop time exc.");
    if(error == 6)lcd.print("Take-off error");
    if(error == 10)lcd.print("Take-off thr.");

  }

  if(last_receive + 3000 < millis() && receive_start_detect && telemetry_lost == 0 && key_press_timer < 200){
    telemetry_lost = 1;
    lcd.clear();
    receive_start_detect = 0;
    page = 100;
  }

  if(error && alarm_sound == 0){
    alarm_sound = 1;
    page = 100 + error;
  }

  if(error == 0 && alarm_sound){
    alarm_sound = 0;
    page = 0;
  }

  if((telemetry_lost == 1 || alarm_sound == 1) && next_sound < millis()){
    digitalWrite(2, HIGH);
    delay(10);
    digitalWrite(2, LOW);
    delay(50);
    digitalWrite(2, HIGH);
    delay(10);
    digitalWrite(2, LOW);
    next_sound = millis() + 1000;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect){
  if(button_store == -1)button_store = analogRead(0);
  if(button_store > 1000)button_store = -1;
}

//When there are two start signatures received the received data between them can be tested to se if it is a valid data stream.
void get_data(void){
  check_byte = 0;                                                                         //Reset the check_byte variabel.
  for(temp_byte=0;temp_byte <= 30; temp_byte ++)check_byte ^= receive_buffer[temp_byte];  //Calculate the check_byte.
  if(check_byte == receive_buffer[31]){                                                   //If the calculated check_byte and the received check_byte are the same.
    if(telemetry_lost > 0){                                                               //If the telemetry signal was lost.
      telemetry_lost = 0;                                                                 //Reset the telemetry lost signal because a valid data stream is received.
      page = 0;                                                                           //Start at page 1.
    }
    last_receive = millis();                                                              //Remember when this reception has arived.
    receive_start_detect = 1;                                                             //Reset the receive_start_detect variable to 1.
    //In the following lines the different variables are restored from the valid data stream.
    //The name of the variables are the same as in the YMFC-32 flight controller program.
    error = receive_buffer[0];
    flight_mode = receive_buffer[1];
    battery_voltage = (float)receive_buffer[2]/10.0;
    temperature = receive_buffer[3] | receive_buffer[4] << 8;
    roll_angle = receive_buffer[5] - 100;
    pitch_angle = receive_buffer[6] - 100;
    start = receive_buffer[7];
    altitude_meters = (receive_buffer[8] | receive_buffer[9] << 8) - 1000;
    if(altitude_meters > max_altitude_meters)max_altitude_meters = altitude_meters;
    takeoff_throttle = receive_buffer[10] | receive_buffer[11] << 8;
    actual_compass_heading = receive_buffer[12] | receive_buffer[13] << 8;
    heading_lock = receive_buffer[14];
    number_used_sats = receive_buffer[15];
    fix_type = receive_buffer[16];
    l_lat_gps = (int32_t)receive_buffer[17] | (int32_t)receive_buffer[18] << 8 | (int32_t)receive_buffer[19] << 16 | (int32_t)receive_buffer[20] << 24;
    l_lon_gps = (int32_t)receive_buffer[21] | (int32_t)receive_buffer[22] << 8 | (int32_t)receive_buffer[23] << 16 | (int32_t)receive_buffer[24] << 24;
    adjustable_setting_1 = (float)(receive_buffer[25] | receive_buffer[26] << 8)/100.0;
    adjustable_setting_2 = (float)(receive_buffer[27] | receive_buffer[28] << 8)/100.0;
    adjustable_setting_3 = (float)(receive_buffer[29] | receive_buffer[30] << 8)/100.0;

    if(number_used_sats >= 4){
      lat_distance = abs(l_lat_gps - l_lat_gps_previous);
      lat_distance *= cos(((float)l_lat_gps/1000000.0) * 0.017453);

      lon_distance = abs(l_lon_gps - l_lon_gps_previous);
      lon_distance = sqrt((lon_distance * lon_distance) + (lat_distance * lat_distance));
      lon_distance /= 1.2626263;
      if(start == 0)lon_distance = 0;

      if(lon_distance < 250){
        speed_loop_counter ++;
        speed_buffer[3] = speed_buffer[2];
        speed_buffer[2] = speed_buffer[1];
        speed_buffer[1] = speed_buffer[0];
        speed_buffer[0] = lon_distance;

        if(speed_loop_counter == 3){
          speed_buffer[4] = speed_buffer[3] + speed_buffer[2] + speed_buffer[1] + speed_buffer[0];
          speed_buffer[4] /= 4;

          speed_loop_counter = 0;
          speed_kmph = speed_buffer[4];
          if(max_speed < speed_kmph)max_speed = speed_kmph;
        }

      }
      //Serial.println(speed_kmph);

      l_lat_gps_previous = l_lat_gps;
      l_lon_gps_previous = l_lon_gps;



    }
  }
}



