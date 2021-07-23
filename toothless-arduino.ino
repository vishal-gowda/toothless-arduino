#include <Wire.h>

#define gyro_address 0xD0

float pid_p_gain_roll = 1.0;  //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.02; //Gain setting for the roll I-controller
float pid_d_gain_roll = 10;   //Gain setting for the roll D-controller
int pid_max_roll = 400;       //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll; //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll; //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll; //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;         //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4;    //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02; //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;    //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;       //Maximum output of the PID-controller (+/-)

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

byte channel_1_captured, channel_2_captured, channel_3_captured, channel_4_captured;
volatile int channel_1, channel_2, channel_3, channel_4;
int channel_1_start, channel_2_start, channel_3_start, channel_4_start;

int start;
int temperature;

float roll_level_adjust, pitch_level_adjust;
int acc_x, acc_y, acc_z, gyro_pitch, gyro_roll, gyro_yaw, acc_total_vector;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal, acc_x_cal, acc_y_cal;
int manual_gyro_pitch_cal_value = 0;
int manual_gyro_roll_cal_value = 0;
int manual_gyro_yaw_cal_value = 0;
int manual_acc_pitch_cal_value = 0;
int manual_acc_roll_cal_value = 0;

int throttle, battery_voltage;
int esc_1, esc_2, esc_3, esc_4;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer, current_time;
unsigned long loop_timer;

void setup()
{
  //Serial.begin(57600)
  Wire.begin();
  TWBR = 12;         //Set the I2C clock speed to 400kHz.
  DDRD |= B11110000; //Configure digital poort 4, 5, 6 and 7 as output
  DDRB |= B00110000; //Configure digital poort 12 and 13 as output.
  digitalWrite(LED_BUILTIN, HIGH);
  gyro_setup();

  PCICR |= (1 << PCIE0);   //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0); //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1); //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2); //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3); //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  battery_voltage = (analogRead(0) + 65) * 1.2317;
  digitalWrite(LED_BUILTIN, LOW);
  loop_timer = micros();
}

void loop()
{
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);    //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3); //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);       //Gyro pid input is deg/sec.

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611; //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;   //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066); //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066); //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector)
  {                                                                   //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296; //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector)
  {                                                                   //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296; //Calculate the roll angle.
  }

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 1.9463; //Accelerometer calibration value for pitch.
  angle_roll_acc += 0.27;    //Accelerometer calibration value for roll.

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;    //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15; //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;   //Calculate the roll angle correction

  //For starting the motors: throttle low and yaw left (step 1).
  if (channel_3 < 1050 && channel_4 < 1050)
    start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1450)
  {
    start = 2;
    digitalWrite(LED_BUILTIN, LOW);

    angle_pitch = angle_pitch_acc; //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && channel_3 < 1050 && channel_4 > 1950)
  {
    start = 0;
    digitalWrite(LED_BUILTIN, HIGH);
  }

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_1 > 1508)
    pid_roll_setpoint = channel_1 - 1508;
  else if (channel_1 < 1492)
    pid_roll_setpoint = channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust; //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;               //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_2 > 1508)
    pid_pitch_setpoint = channel_2 - 1508;
  else if (channel_2 < 1492)
    pid_pitch_setpoint = channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust; //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_3 > 1050)
  { //Do not yaw when turning off the motors.
    if (channel_4 > 1508)
      pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
    else if (channel_4 < 1492)
      pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
  }

  calculate_pid(); //PID inputs are known. So we can calculate the pid output.

  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if (battery_voltage < 1000 && battery_voltage > 600)
    digitalWrite(LED_BUILTIN, HIGH);

  throttle = channel_3; //We need the throttle signal as a base signal.

  if (start == 2)
  { //The motors are started.
    if (throttle > 1800)
      throttle = 1800;                                                      //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800)
    {                                                            //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage) / (float)3500); //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage) / (float)3500); //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage) / (float)3500); //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage) / (float)3500); //Compensate the esc-4 pulse for voltage drop.
    }

    if (esc_1 < 1100)
      esc_1 = 1100; //Keep the motors running.
    if (esc_2 < 1100)
      esc_2 = 1100; //Keep the motors running.
    if (esc_3 < 1100)
      esc_3 = 1100; //Keep the motors running.
    if (esc_4 < 1100)
      esc_4 = 1100; //Keep the motors running.

    if (esc_1 > 2000)
      esc_1 = 2000; //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)
      esc_2 = 2000; //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)
      esc_3 = 2000; //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)
      esc_4 = 2000; //Limit the esc-4 pulse to 2000us.
  }
  else
  {
    esc_1 = 1000; //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000; //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000; //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000; //If start is not 2 keep a 1000us pulse for ess-4.
  }

  if (micros() - loop_timer > 4050)
    digitalWrite(LED_BUILTIN, HIGH); //Turn on the LED if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000)
    ;                    //We wait until 4000us are passed.
  loop_timer = micros(); //Set the timer for the next loop.

  PORTD |= B11110000;                   //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer; //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer; //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer; //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer; //Calculate the time of the faling edge of the esc-4 pulse.

  gyro_read();

  while (PORTD >= 16)
  {                            //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros(); //Read the current time.
    if (timer_channel_1 <= esc_loop_timer)
      PORTD &= B11101111; //Set digital output 4 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer)
      PORTD &= B11011111; //Set digital output 5 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer)
      PORTD &= B10111111; //Set digital output 6 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer)
      PORTD &= B01111111; //Set digital output 7 to low if the time is expired.
  }
}

ISR(PCINT0_vect)
{
  current_time = micros();
  //Channel 1=========================================
  if (PINB & B00000001)
  { //Is input 8 high?
    if (channel_1_captured == 0)
    {                                 //Input 8 changed from 0 to 1.
      channel_1_captured = 1;         //Remember current input state.
      channel_1_start = current_time; //Set timer_1 to current_time.
    }
  }
  else if (channel_1_captured == 1)
  {                                             //Input 8 is not high and changed from 1 to 0.
    channel_1_captured = 0;                     //Remember current input state.
    channel_1 = current_time - channel_1_start; //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if (PINB & B00000010)
  { //Is input 9 high?
    if (channel_2_captured == 0)
    {                                 //Input 9 changed from 0 to 1.
      channel_2_captured = 1;         //Remember current input state.
      channel_2_start = current_time; //Set timer_2 to current_time.
    }
  }
  else if (channel_2_captured == 1)
  {                                             //Input 9 is not high and changed from 1 to 0.
    channel_2_captured = 0;                     //Remember current input state.
    channel_2 = current_time - channel_2_start; //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if (PINB & B00000100)
  { //Is input 10 high?
    if (channel_3_captured == 0)
    {                                 //Input 10 changed from 0 to 1.
      channel_3_captured = 1;         //Remember current input state.
      channel_3_start = current_time; //Set timer_3 to current_time.
    }
  }
  else if (channel_3_captured == 1)
  {                                             //Input 10 is not high and changed from 1 to 0.
    channel_3_captured = 0;                     //Remember current input state.
    channel_3 = current_time - channel_3_start; //Channel 3 is current_time - timer_3.
  }
  //Channel 4=========================================
  if (PINB & B00001000)
  { //Is input 11 high?
    if (channel_4_captured == 0)
    {                                 //Input 11 changed from 0 to 1.
      channel_4_captured = 1;         //Remember current input state.
      channel_4_start = current_time; //Set timer_4 to current_time.
    }
  }
  else if (channel_4_captured == 1)
  {                                             //Input 11 is not high and changed from 1 to 0.
    channel_4_captured = 0;                     //Remember current input state.
    channel_4 = current_time - channel_4_start; //Channel 4 is current_time - timer_4.
  }
}

void gyro_setup(void)
{
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
  Wire.write(0x6B);                     //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                     //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();               //End the transmission with the gyro.

  Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
  Wire.write(0x1B);                     //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                     //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();               //End the transmission with the gyro

  Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
  Wire.write(0x1C);                     //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                     //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();               //End the transmission with the gyro

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search
  Wire.write(0x1B);                     //Start reading @ register 0x1B
  Wire.endTransmission();               //End the transmission
  Wire.requestFrom(gyro_address, 1);    //Request 1 bytes from the gyro
  while (Wire.available() < 1)
    ; //Wait until the 6 bytes are received
  if (Wire.read() != 0x08)
  { //Check if the value is 0x08
    error_handler();
  }

  Wire.beginTransmission(gyro_address); //Start communication with the address found during search
  Wire.write(0x1A);                     //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                     //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();               //End the transmission with the gyro

  for (int cal_int = 0; cal_int < 2000; cal_int++)
  { //Take 2000 readings for calibration.
    if (cal_int % 25 == 0)
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); //Change the led status to indicate calibration.
    gyro_read();                                            //Read the gyro output.
    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw;
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;      //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000); //Wait 1000us.
    PORTD &= B00001111;      //Set digital poort 4, 5, 6 and 7 low.
    delay(4);                //Wait 4 milliseconds before the next loop.
  }

  acc_x_cal /= 2000;
  acc_y_cal /= 2000;
  gyro_roll_cal /= 2000;
  gyro_pitch_cal /= 2000;
  gyro_yaw_cal /= 2000;
  manual_acc_pitch_cal_value = acc_y_cal;
  manual_acc_roll_cal_value = acc_x_cal;
  manual_gyro_pitch_cal_value = gyro_pitch_cal;
  manual_gyro_roll_cal_value = gyro_roll_cal;
  manual_gyro_yaw_cal_value = gyro_yaw_cal;
}

void gyro_read(void)
{
  Wire.beginTransmission(gyro_address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 14);

  while (Wire.available() < 14)
    ;
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_roll = Wire.read() << 8 | Wire.read();
  gyro_pitch = Wire.read() << 8 | Wire.read();
  gyro_yaw = Wire.read() << 8 | Wire.read();
  gyro_pitch *= -1;
  gyro_yaw *= -1;

  acc_y -= manual_acc_pitch_cal_value;
  acc_x -= manual_acc_roll_cal_value;
  gyro_roll -= manual_gyro_roll_cal_value;
  gyro_pitch -= manual_gyro_pitch_cal_value;
  gyro_yaw -= manual_gyro_yaw_cal_value;
}

void calculate_pid(void)
{
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)
    pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)
    pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)
    pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)
    pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)
    pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)
    pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)
    pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)
    pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)
    pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void error_handler()
{
  while (1)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(1000);
  }
}
