
int16_t receiver_adjust = 63;
const float pid_p_gain_roll = 1.0;               //Gain setting for the pitch and roll P-controller (default = 1.3).
const float pid_i_gain_roll = 0.02;              //Gain setting for the pitch and roll I-controller (default = 0.04).
const float pid_d_gain_roll = 10.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
const int16_t pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int16_t pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.04;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int16_t pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).


volatile uint32_t us;

int16_t channel_1_start = 0;
int16_t channel_1 = 0;
uint8_t channel_1_captured = 0;
int16_t channel_2_start = 0;
int16_t channel_2 = 0;
uint8_t channel_2_captured = 0;
int16_t channel_3_start = 0;
int16_t channel_3 = 0;
uint8_t channel_3_captured = 0;
int16_t channel_4_start = 0;
int16_t channel_4 = 0;
uint8_t channel_4_captured = 0;
uint8_t start;

uint32_t loop_timer;
int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle;

int32_t acc_total_vector;
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
