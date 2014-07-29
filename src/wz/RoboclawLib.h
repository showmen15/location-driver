#ifndef ROBOCLAW_H_
#define ROBOCLAW_H_

#include <linux/types.h>
#include <termios.h>

int rc_uart_open(const char *blockdevice);
int rc_uart_init(int fd, speed_t speed);
ssize_t rc_uart_write(int fd, int bytes, __u8 *buf);
ssize_t rc_uart_read(int fd, int to_read, __u8 *buf);
int h_uart_flush_input(int fd);
int rc_uart_close(int fd);

int rc_gpio_open(const char *gpio_path);
int rc_gpio_set(int gpio_fd, bool state);
int rc_gpio_close(int gpio_fd);

int rc_reset(int gpio_fd);

void fill_crc(__u8 *buf, int size);
bool check_crc(__u8 rc_address, __u8 command_id, __u8 *buf, int size);

#define RC_ERROR_NORMAL 0x00
#define RC_ERROR_M1_OVERCURRENT 0x01
#define RC_ERROR_M2_OVERCURRENT 0x02
#define RC_ERROR_ESTOP 0x04
#define RC_ERROR_TEMPERATURE 0x08
#define RC_ERROR_MAIN_BATTERY_HIGH 0x10
#define RC_ERROR_MAIN_BATTERY_LOW 0x20
#define RC_ERROR_LOGIC_BATTERY_HIGH 0x40
#define RC_ERROR_LOGIC_BATTERY_LOW 0x80

// 0 - Drive Forward M1
int rc_drive_forward_m1(int fd, __u8 rc_address, __u8 speed); 

// 1 - Drive Backwards M1
int rc_drive_backwards_m1(int fd, __u8 rc_address, __u8 speed); 

// 2 - Set Minimum Main Voltage
int rc_set_minimum_main_voltage(int fd, __u8 rc_address, __u8 voltage);

// 3 - Set Maximum Main Voltage
int rc_set_maximum_main_voltage(int fd, __u8 rc_address, __u8 voltage);

// 4 - Drive Forward M2
int rc_drive_forward_m2(int fd, __u8 rc_address, __u8 speed);

// 5 - Drive Backwards M2
int rc_drive_backwards_m2(int fd, __u8 rc_address, __u8 speed);

// 6 - Drive M1 (7 Bit)
int rc_drive_m1(int fd, __u8 rc_address, __u8 speed);

// 7 - Drive M2 (7 Bit)
int rc_drive_m2(int fd, __u8 rc_address, __u8 speed);

// 8 - Drive Forward
int rc_drive_forward(int fd, __u8 rc_address, __u8 speed);

// 9 - Drive Backwards
int rc_drive_backwards(int fd, __u8 rc_address, __u8 speed);

// 10 - Turn right
int rc_turn_right(int fd, __u8 rc_address, __u8 value);

// 11 - Turn left
int rc_turn_left(int fd, __u8 rc_address, __u8 value);

// 12 - Drive Forward or Backward (7 Bit)
int rc_drive(int fd, __u8 rc_address, __u8 speed);

// 13 - Turn Left or Right (7 Bit)
int rc_turn(int fd, __u8 rc_address, __u8 value);


// 21 - Read Firmware Version
int rc_read_firmware_version(int fd, __u8 rc_address, unsigned char *str);

// 24 - Read Main Battery Voltage Level
int rc_read_main_battery_voltage_level(int fd, __u8 rc_address, __u16 *value);

// 25 - Read Logic Battery Voltage Level
int rc_read_logic_battery_voltage_level(int fd, __u8 rc_address, __u16 *value);

// 26 - Set Minimum Logic Voltage Level
int rc_set_minimum_logic_voltage_level(int fd, __u8 rc_address, __u8 voltage);

// 27 - Set Maximum Logic Voltage Level
int rc_set_maximum_logic_voltage_level(int fd, __u8 rc_address, __u8 voltage);

// 16 - Read Quadrature Encoder Register M1
int rc_read_encoder_register_m1(int fd, __u8 rc_address, __u32 *value, __u8 *status);

// 17 - Read Quadrature Encoder Register M2
int rc_read_encoder_register_m2(int fd, __u8 rc_address, __u32 *value, __u8 *status);


//18 - Read Speed M1
int rc_read_speed_m1(int fd, __u8 rc_address, __u32 *value, __u8 *direction);

// 19 - Read Speed M2
int rc_read_speed_m2(int fd, __u8 rc_address, __u32 *value, __u8 *direction);

// 20 - Reset Quadrature Encoder Counters
int rc_reset_encoder_counters(int fd, __u8 rc_address);

// 28 - Set PID Constants M1
int rc_set_pid_consts_m1(int fd, __u8 rc_address, __u32 d, __u32 p, __u32 i, __u32 qpps);

// 29 - Set PID Constants M2
int rc_set_pid_consts_m2(int fd, __u8 rc_address, __u32 d, __u32 p, __u32 i, __u32 qpps);

// 30 - Read Current Speed M1
int rc_read_speed125_m1(int fd, __u8 rc_address, __u32 *value);

// 31 - Read Current Speed M2
int rc_read_speed125_m2(int fd, __u8 rc_address, __u32 *value);
//int rc_drive_m1_duty_cycle(int fd, __u8 rc_address, __s16 duty);
//int rc_drive_m2_duty_cycle(int fd, __u8 rc_address, __s16 duty);
//int rc_drive_duty_cycle(int fd, __u8 rc_address, __s16 duty_m1, __s16 duty_m2);

// 35 - Drive M1 With Signed Speed
int rc_drive_m1_speed(int fd, __u8 rc_address, __s32 speed);

// 36 - Drive M2 With Signed Speed
int rc_drive_m2_speed(int fd, __u8 rc_address, __s32 speed);

// 37 - Mix Mode Drive M1 / M2 With Signed Speed
int rc_drive_speed(int fd, __u8 rc_address, __s32 speed_m1, __s32 speed_m2);

// 38 - Drive M1 With Signed Speed And Acceleration
int rc_drive_m1_speed_accel(int fd, __u8 rc_address, __u32 accel, __s32 speed);

// 39 - Drive M2 With Signed Speed And Acceleration
int rc_drive_m2_speed_accel(int fd, __u8 rc_address, __u32 accel, __s32 speed);

// 40 - Mix Mode Drive M1 / M2 With Speed And Acceleration
int rc_drive_speed_accel(int fd, __u8 rc_address, __u32 accel, __s32 speed_m1, __s32 speed_m2);

// 41 - Buffered M1 Drive With Signed Speed And Distance
int rc_buffered_m1_drive_speed_dist(int fd, __u8 rc_address,__s32 speed, __u32 dist, __u8 now);

// 42 - Buffered M2 Drive With Signed Speed And Distance
int rc_buffered_m2_drive_speed_dist(int fd, __u8 rc_address,__s32 speed, __u32 dist, __u8 now);

// 43 - Buffered Mix Mode Drive M1 / M2 With Signed Speed And Distance
int rc_buffered_drive_speed_dist(int fd, __u8 rc_address, __s32 speed_m1, __u32 dist_m1, __s32 speed_m2, __u32 dist_m2, __u8 now);


// 44 - Buffered M1 Drive With Signed Speed, Accel And Distance
int rc_buffered_m1_drive_speed_accel_dist(int fd, __u8 rc_address, __u32 accel, __s32 speed, __u32 dist, __u8 now);

// 45 - Buffered M2 Drive With Signed Speed, Accel And Distance
int rc_buffered_m2_drive_speed_accel_dist(int fd, __u8 rc_address, __u32 accel, __s32 speed, __u32 dist, __u8 now);

// 46 - Buffered Mix Mode Drive M1 / M2 With Signed Speed, Accel And Distance
int rc_buffered_drive_speed_accel_dist(int fd, __u8 rc_address, __u32 accel, __s32 speed_m1, __u32 dist_m1, __s32 speed_m2, __u32 dist_m2, __u8 now);

// 55 - Read Motor 1 P, I, D and QPPS Settings
int rc_read_pid_const_m1(int fd, __u8 rc_address, __u32 *d, __u32 *p, __u32 *i, __u32 *qpps); 

// 56 - Read Motor 2 P, I, D and QPPS Settings
int rc_read_pid_const_m2(int fd, __u8 rc_address, __u32 *d, __u32 *p, __u32 *i, __u32 *qpps); 

// 82 Read Temperature
int rc_read_temperature(int fd, __u8 rc_address, __u16* value);

// 90 - Read Error Status
int rc_read_error_status(int fd, __u8 rc_address, __u8* error);

// 94 - Write Settings to EEPROM
int rc_write_to_eeprom(int fd, __u8 rc_address);

#define CRC(x) static_cast<__u8>((x) & 0x7F)
#define BYTE(x, n) static_cast<__u8>((x >> (8 * n)) & 0xFF)

/*
* Standard commands
*/
#define DRIVE_FORWARD_M1 0
#define DRIVE_BACKWARDS_M1 1
#define SET_MINIMUM_MAIN_VOLTAGE 2
#define SET_MAXIMUM_MAIN_VOLTAGE 3
#define DRIVE_FORWARD_M2 4
#define DRIVE_BACKWARDS_M2 5
#define DRIVE_M1 6
#define DRIVE_M2 7

/*
* Mix mode commands
*/
#define DRIVE_FORWARD 8
#define DRIVE_BACKWARDS 9
#define TURN_RIGHT 10
#define TURN_LEFT 11
#define DRIVE_FORWARD_OR_BACKWARD 12
#define TURN_LEFT_OR_RIGHT 13

/*
* Battery and firmware version commands
*/

#define READ_FIRMWARE_VERSION 21
#define READ_MAIN_BATTEY_VOLTAGE_LEVEL 24
#define READ_LOGIC_BATTERY_VOLTAGE_LEVEL 25
#define SET_MINIMUM_LOGIC_VOLTAGE_LEVEL 26
#define SET_MAXIMUM_LOGIC_VOLTAGE_LEVEL 27

/*
* Reading quadrature encoders
*/
#define READ_QUADRATURE_ENCODER_REGISTER_M1 16
#define READ_QUADRATURE_ENCODER_REGISTER_M2 17
#define READ_SPEED_M1 18
#define READ_SPEED_M2 19
#define RESET_QUADRATURE_ENCODER_COUNTERS 20

/*
* Motor control by quadrature encoders
*/
#define SET_PID_CONSTANTS_M1 28
#define SET_PID_CONSTANTS_M2 29
#define READ_CURRENT_SPEED_M1 30
#define READ_CURRENT_SPEED_M2 31
#define DRIVE_M1_DUTY_CYCLE 32
#define DRIVE_M2_DUTY_CYCLE 33
#define MIX_MODE_DRIVE_DUTY_CYCLE 34
#define DRIVE_M1_SPEED 35
#define DRIVE_M2_SPEED 36
#define MIX_MODE_DRIVE_SPEED 37
#define DRIVE_M1_SPEED_ACCEL 38
#define DRIVE_M2_SPEED_ACCEL 29
#define MIX_MODE_DRIVE_SPEED_ACCEL 40
#define BUFFERED_M1_DRIVE_SPEED_DIST 41
#define BUFFERED_M2_DRIVE_SPEED_DIST 42
#define BUFFERED_MIX_MODE_DRIVE_SPEED_DIST 43
#define BUFFERED_M1_DRIVE_SPEED_ACCEL_DIST 44
#define BUFFERED_M2_DRIVE_SPEED_ACCEL_DIST 45
#define BUFFERED_MIX_MODE_SPEED_ACCEL_DISTANCE 46
#define READ_BUFFER_LENGTH 47
#define SET_PWM_RESOLUTION 48

/*
* Other
*/

#define READ_TEMPERATURE 82
#define READ_ERROR_STATUS 90
#define READ_PID_CONST_M1 55
#define READ_PID_CONST_M2 56
#define WRITE_TO_EEPROM 94

#endif
