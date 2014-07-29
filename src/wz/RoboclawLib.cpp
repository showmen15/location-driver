#include <cstdio>
#include <unistd.h>
#include <linux/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <strings.h>

#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>

#include "RoboclawLib.h"

void fill_crc(__u8 *buf, int size) {
	__u8 sum = 0;

	for (int i = 0; i < size - 1; i++) {
		sum = (__u8)(sum + buf[i]);
	}

	buf[size - 1] = sum & 0x7F;
}

bool check_crc(__u8 rc_address, __u8 command_id, __u8 *buf, int size) {
    __u8 sum = (__u8)(rc_address + command_id);

    for (int i = 0; i < size - 1; i++) {
        sum = (__u8)(sum + buf[i]);
    }

    return buf[size - 1] == (sum & 0x7F);
}

int rc_uart_open(const char *blockdevice) {
	return open(blockdevice, O_RDWR);
}

int rc_uart_close(int fd) {
	return close(fd);
}

int rc_uart_init(int fd, speed_t speed)
{
    struct termios termios;

    bzero(&termios, sizeof(termios));
    cfmakeraw(&termios);

    termios.c_cflag |= (CS8 | CREAD | CLOCAL);

    termios.c_cc[VMIN] = 0;
    termios.c_cc[VTIME] = 1;

    cfsetispeed(&termios, speed);
    cfsetospeed(&termios, speed);

    return tcsetattr(fd, TCSANOW, &termios);
}

ssize_t rc_uart_write(int fd, int bytes, __u8 *buf) {
    ssize_t res, chars_in_tx_queue;

    res = write(fd, buf, bytes);
    if (res < 0) {
        return -1;
    }

    // wait for buffer to empty
    do {
    	boost::this_thread::sleep(boost::posix_time::microseconds(500));
    	ioctl (fd, TIOCOUTQ, &chars_in_tx_queue);
    } while (chars_in_tx_queue > 0);

    //tcdrain(fd);

    return res;
}

ssize_t rc_uart_read(int fd, int to_read, __u8 *buf) {
	ssize_t received;
    ssize_t sum = 0;

    while (sum < to_read) {
    	received = read(fd, buf, 1);
    	buf += received;
    	sum += received;

    	if (received <= 0) {
    		return sum;
    	}
    }

    return sum;
}

int h_uart_flush_input(int fd) {
     return tcflush(fd, TCIFLUSH);
}


int rc_gpio_open(const char *gpio_path) {
    return open(gpio_path, O_WRONLY);
}

int rc_gpio_set(int gpio_fd, bool state) {
    const char *command;

    command = state ? "1" : "0";
   
    ssize_t res = write(gpio_fd, command, 1);
    if (res < 0) {
        return -1;
    }

    return 0;
}

int rc_gpio_close(int gpio_fd) {
    return close(gpio_fd);
}

int rc_reset(int gpio_fd) {

    if (rc_gpio_set(gpio_fd, false) < 0) {
        return -1;
    }

    boost::this_thread::sleep(boost::posix_time::microseconds(500));
    
    if (rc_gpio_set(gpio_fd, true) < 0) {
        return -1;
    }

    return 0;
}


int rc_drive_forward_m1(int fd, __u8 rc_address, __u8 speed) {
	const ssize_t command_size = 4;

    __u8 buffer[command_size] = {
    		rc_address,
    		DRIVE_FORWARD_M1,
    		speed,
    		0x00, // to be filled with CRC
    };
    fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}

    return 0;
} 

int rc_drive_backwards_m1(int fd, __u8 rc_address, __u8 speed) {
	const ssize_t command_size = 4;

	__u8 buffer[command_size] = {
    		rc_address,
    		DRIVE_BACKWARDS_M1,
    		speed,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_set_minimum_main_voltage(int fd, __u8 rc_address, __u8 voltage) {
	const ssize_t command_size = 4;
	
	__u8 buffer[command_size] = {
			rc_address,
			SET_MINIMUM_MAIN_VOLTAGE,
			voltage,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_set_maximum_main_voltage(int fd, __u8 rc_address, __u8 voltage) {
	const ssize_t command_size = 4;

	__u8 buffer[command_size] = {
			rc_address,
			SET_MAXIMUM_MAIN_VOLTAGE,
			voltage,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_drive_forward_m2(int fd, __u8 rc_address, __u8 speed) {
	const ssize_t command_size = 4;

    __u8 buffer[command_size] = {
    		rc_address,
    		DRIVE_FORWARD_M2,
    		speed,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_drive_backwards_m2(int fd, __u8 rc_address, __u8 speed) {
	const ssize_t command_size = 4;

	__u8 buffer[command_size] = {
    		rc_address,
    		DRIVE_BACKWARDS_M2,
    		speed,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_drive_m1(int fd, __u8 rc_address, __u8 speed) {
	const ssize_t command_size = 4;

	__u8 buffer[command_size] = {
    		rc_address,
    		DRIVE_M1,
    		speed,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_drive_m2(int fd, __u8 rc_address, __u8 speed) {
	const ssize_t command_size = 4;

	__u8 buffer[command_size] = {
    		rc_address,
    		DRIVE_M2,
    		speed,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_drive_forward(int fd, __u8 rc_address, __u8 speed) {
    const ssize_t command_size = 4;

    __u8 buffer[command_size] = {
    		rc_address,
    		DRIVE_FORWARD,
    		speed,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_drive_backwards(int fd, __u8 rc_address, __u8 speed) {
    const ssize_t command_size = 4;

    __u8 buffer[command_size] = {
    		rc_address,
    		DRIVE_BACKWARDS,
    		speed,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_turn_right(int fd, __u8 rc_address, __u8 value) {
	const ssize_t command_size = 4;

    __u8 buffer[command_size] = {
    		rc_address,
    		TURN_RIGHT,
    		value,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_turn_left(int fd, __u8 rc_address, __u8 value) {
    const ssize_t command_size = 4;

    __u8 buffer[command_size] = {
    		rc_address,
    		TURN_LEFT,
    		value,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}


int rc_drive(int fd, __u8 rc_address, __u8 speed) {
	const ssize_t command_size = 4;

    __u8 buffer[command_size] = {
    		rc_address,
    		DRIVE_FORWARD_OR_BACKWARD,
    		speed,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_turn(int fd, __u8 rc_address, __u8 value) {
	const ssize_t command_size = 4;

    __u8 buffer[command_size] = {
    		rc_address,
    		TURN_LEFT_OR_RIGHT,
    		value,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_read_firmware_version(int fd, __u8 rc_address, unsigned char *str) {
    __u8 buffer[4] = {
    		rc_address,
    		READ_FIRMWARE_VERSION};

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, 2, buffer) != 2) {
    	return -1;
    }

    ssize_t received;
    int sum = 0;
    unsigned char *str_t = str;

    bool one_more = false;
    while((received = read(fd, str_t, 1)) > 0) {
    	sum++;

        if (one_more) {
            break;
        } else if(*str_t == 0x00) {
    		one_more = true;
    	}

    	str_t++;
    }

    if (received == -1 || !check_crc(rc_address, READ_FIRMWARE_VERSION, str, sum)) {
        return -1;
    }

    return sum - 1;
}

int rc_read_main_battery_voltage_level(int fd, __u8 rc_address, __u16 *value) {
	const ssize_t command_size = 2;

    __u8 buffer[4] = {
    		rc_address,
    		READ_MAIN_BATTEY_VOLTAGE_LEVEL};

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1;
    }

    const ssize_t reply_size = 3;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
        !check_crc(rc_address, READ_MAIN_BATTEY_VOLTAGE_LEVEL, in_buffer, reply_size)) {
    	return -1;
    }

    *value = (__u16)((in_buffer[0] << 8) | in_buffer[1]);

    return 0;
}

int rc_read_logic_battery_voltage_level(int fd, __u8 rc_address, __u16 *value) {
    __u8 buffer[4] = {
    		rc_address,
    		READ_LOGIC_BATTERY_VOLTAGE_LEVEL};

    h_uart_flush_input(fd);

    rc_uart_write(fd, 2, buffer);

    const ssize_t reply_size = 3;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
         !check_crc(rc_address, READ_LOGIC_BATTERY_VOLTAGE_LEVEL, in_buffer, reply_size)) {
    	return -1;
    }

    *value = (__u16)((in_buffer[0] << 8) | in_buffer[1]);

    return 0;
}

int rc_set_minimum_logic_voltage_level(int fd, __u8 rc_address, __u8 voltage) {
	const ssize_t command_size = 4;

	__u8 buffer[command_size] = {
			rc_address,
			SET_MINIMUM_LOGIC_VOLTAGE_LEVEL,
			voltage,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_set_maximum_logic_voltage_level(int fd, __u8 rc_address, __u8 voltage) {
	const ssize_t command_size = 4;

	__u8 buffer[command_size] = {
			rc_address,
			SET_MAXIMUM_LOGIC_VOLTAGE_LEVEL,
			voltage,
    		0x00, // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_read_encoder_register_m1(int fd, __u8 rc_address, __u32 *value, __u8 *status) {
	const ssize_t command_size = 2;

    __u8 buffer[2] = {
    		rc_address,
    		READ_QUADRATURE_ENCODER_REGISTER_M1};

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1;
    }

    const ssize_t reply_size = 6;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
        !check_crc(rc_address, READ_QUADRATURE_ENCODER_REGISTER_M1, in_buffer, reply_size)) {
    	return -1;
    }

    *value = (in_buffer[0] << 24) | (in_buffer[1] << 16) | (in_buffer[2] << 8) | in_buffer[3];
    *status = in_buffer[4];

    return 0;
}

int rc_read_encoder_register_m2(int fd, __u8 rc_address, __u32 *value, __u8 *status) {
	const ssize_t command_size = 2;

    __u8 buffer[4] = {
    		rc_address,
    		READ_QUADRATURE_ENCODER_REGISTER_M2};

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1;
    }

    const ssize_t reply_size = 6;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
        !check_crc(rc_address, READ_QUADRATURE_ENCODER_REGISTER_M2, in_buffer, reply_size)) {
    	return -1;
    }

	*value = (in_buffer[0] << 24) | (in_buffer[1] << 16) | (in_buffer[2] << 8) | in_buffer[3];
	*status = in_buffer[4];

	return 0;
}

int rc_read_speed_m1(int fd, __u8 rc_address, __u32 *value, __u8 *direction) {
	const ssize_t command_size = 2;

    __u8 buffer[2] = {
    		rc_address,
    		READ_SPEED_M1};

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1;
    }

    const ssize_t reply_size = 6;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size || 
        !check_crc(rc_address, READ_SPEED_M1, in_buffer, reply_size)) {
    	return -1;
    }

	*value = (in_buffer[0] << 24) | (in_buffer[1] << 16) | (in_buffer[2] << 8) | in_buffer[3];
	*direction = in_buffer[4];

	return 0;
}


int rc_read_speed_m2(int fd, __u8 rc_address, __u32 *value, __u8 *direction) {
	const ssize_t command_size = 2;

    __u8 buffer[2] = {
    		rc_address,
    		READ_SPEED_M2};

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1;
    }

    const ssize_t reply_size = 6;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size || 
        !check_crc(rc_address, READ_SPEED_M2, in_buffer, reply_size)) {
    	return -1;
    }

	*value = (in_buffer[0] << 24) | (in_buffer[1] << 16) | (in_buffer[2] << 8) | in_buffer[3];
	*direction = in_buffer[4];

	return 0;
}

int rc_set_pid_consts_m1(int fd, __u8 rc_address, __u32 d, __u32 p, __u32 i, __u32 qpps) {
	const ssize_t command_size = 19;

     __u8 buffer[command_size] = {
	     rc_address,
	     SET_PID_CONSTANTS_M1,

	     BYTE(d, 3),
	     BYTE(d, 2),
	     BYTE(d, 1),
	     BYTE(d, 0),

	     BYTE(p, 3),
	     BYTE(p, 2),
	     BYTE(p, 1),
	     BYTE(p, 0),

	     BYTE(i, 3),
	     BYTE(i, 2),
	     BYTE(i, 1),
	     BYTE(i, 0),

	     BYTE(qpps, 3),
	     BYTE(qpps, 2),
	     BYTE(qpps, 1),
	     BYTE(qpps, 0),

	     0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_set_pid_consts_m2(int fd, __u8 rc_address, __u32 d, __u32 p, __u32 i, __u32 qpps) {
	const ssize_t command_size = 19;
    
     __u8 buffer[command_size] = {
	     rc_address,
	     SET_PID_CONSTANTS_M2,

	     BYTE(d, 3),
	     BYTE(d, 2),
	     BYTE(d, 1),
	     BYTE(d, 0),

	     BYTE(p, 3),
	     BYTE(p, 2),
	     BYTE(p, 1),
	     BYTE(p, 0),

	     BYTE(i, 3),
	     BYTE(i, 2),
	     BYTE(i, 1),
	     BYTE(i, 0),

	     BYTE(qpps, 3),
	     BYTE(qpps, 2),
	     BYTE(qpps, 1),
	     BYTE(qpps, 0),

	     0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_reset_encoder_counters(int fd, __u8 rc_address) {
	const ssize_t command_size = 2;

    __u8 buffer[4] = {
    		rc_address,
    		RESET_QUADRATURE_ENCODER_COUNTERS};

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}


int rc_read_speed125_m1(int fd, __u8 rc_address, __u32 *value) {
	const ssize_t command_size = 2;

    __u8 buffer[2] = {
    		rc_address,
    		READ_CURRENT_SPEED_M1};
    
    h_uart_flush_input(fd);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
		return -1;
	}

	const ssize_t reply_size = 4;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
        !check_crc(rc_address, READ_CURRENT_SPEED_M1, in_buffer, reply_size)) {
    	return -1;
    }

	*value = (in_buffer[0] << 24) | (in_buffer[1] << 16) | (in_buffer[2] << 8) | in_buffer[3];

	return 0;
}


int rc_read_speed125_m2(int fd, __u8 rc_address, __u32 *value) {
	const ssize_t command_size = 2;

    __u8 buffer[2] = {
    		rc_address,
    		READ_CURRENT_SPEED_M2};

    h_uart_flush_input(fd);
    
	if (rc_uart_write(fd, command_size, buffer) != command_size) {
		return -1;
	}

	const ssize_t reply_size = 4;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
        !check_crc(rc_address, READ_CURRENT_SPEED_M2, in_buffer, reply_size)) {
    	return -1;
    }

	*value = (in_buffer[0] << 24) | (in_buffer[1] << 16) | (in_buffer[2] << 8) | in_buffer[3];

	return 0;
}


int rc_drive_m1_speed(int fd, __u8 rc_address, __s32 speed_m) {
	const ssize_t command_size = 7;

	__u8 buffer[command_size] = {
		rc_address,
		DRIVE_M1_SPEED,

		BYTE(speed_m, 3),
		BYTE(speed_m, 2),
		BYTE(speed_m, 1),
		BYTE(speed_m, 0),

		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

 	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_drive_m2_speed(int fd, __u8 rc_address, __s32 speed_m) {
	const ssize_t command_size = 7;

	__u8 buffer[command_size] = {
		rc_address,
		DRIVE_M2_SPEED,

		BYTE(speed_m, 3),
		BYTE(speed_m, 2),
		BYTE(speed_m, 1),
		BYTE(speed_m, 0),

		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}


int rc_drive_speed(int fd, __u8 rc_address, __s32 speed_m1, __s32 speed_m2) {
	const ssize_t command_size = 11;

	__u8 buffer[command_size] = {
		rc_address,
		MIX_MODE_DRIVE_SPEED,

		BYTE(speed_m1, 3),
		BYTE(speed_m1, 2),
		BYTE(speed_m1, 1),
		BYTE(speed_m1, 0),

		BYTE(speed_m2, 3),
		BYTE(speed_m2, 2),
		BYTE(speed_m2, 1),
		BYTE(speed_m2, 0),

		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}


int rc_drive_m1_speed_accel(int fd, __u8 rc_address, __u32 accel, __s32 speed) {
	const ssize_t command_size = 11;

	__u8 buffer[command_size] = {
		rc_address,
		DRIVE_M1_SPEED_ACCEL,

		BYTE(accel, 3),
		BYTE(accel, 2),
		BYTE(accel, 1),
		BYTE(accel, 0),

		BYTE(speed, 3),
		BYTE(speed, 2),
		BYTE(speed, 1),
		BYTE(speed, 0),

		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}


int rc_drive_m2_speed_accel(int fd, __u8 rc_address, __u32 accel, __s32 speed) {
	const ssize_t command_size = 11;

	__u8 buffer[command_size] = {
		rc_address,
		DRIVE_M2_SPEED_ACCEL,

		BYTE(accel, 3),
		BYTE(accel, 2),
		BYTE(accel, 1),
		BYTE(accel, 0),

		BYTE(speed, 3),
		BYTE(speed, 2),
		BYTE(speed, 1),
		BYTE(speed, 0),

		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}


int rc_drive_speed_accel(int fd, __u8 rc_address, __u32 accel, __s32 speed_m1, __s32 speed_m2) {
	const ssize_t command_size = 15;

	__u8 buffer[command_size] = {
		rc_address,
		MIX_MODE_DRIVE_SPEED_ACCEL,

		BYTE(accel, 3),
		BYTE(accel, 2),
		BYTE(accel, 1),
		BYTE(accel, 0),

		BYTE(speed_m1, 3),
		BYTE(speed_m1, 2),
		BYTE(speed_m1, 1),
		BYTE(speed_m1, 0),

		BYTE(speed_m2, 3),
		BYTE(speed_m2, 2),
		BYTE(speed_m2, 1),
		BYTE(speed_m2, 0),

		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_buffered_m1_drive_speed_dist(int fd, __u8 rc_address,__s32 speed, __u32 dist, __u8 now) {
	const ssize_t command_size = 12;

	__u8 buffer[command_size] = {
		rc_address,
		BUFFERED_M1_DRIVE_SPEED_DIST,

		BYTE(speed, 3),
		BYTE(speed, 2),
		BYTE(speed, 1),
		BYTE(speed, 0),

		BYTE(dist, 3),
		BYTE(dist, 2),
		BYTE(dist, 1),
		BYTE(dist, 0),

		now,
		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_buffered_m2_drive_speed_dist(int fd, __u8 rc_address,__s32 speed, __u32 dist, __u8 now) {
	const ssize_t command_size = 12;

	__u8 buffer[command_size] = {
		rc_address,
		BUFFERED_M2_DRIVE_SPEED_DIST,

		BYTE(speed, 3),
		BYTE(speed, 2),
		BYTE(speed, 1),
		BYTE(speed, 0),

		BYTE(dist, 3),
		BYTE(dist, 2),
		BYTE(dist, 1),
		BYTE(dist, 0),

		now,
		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_buffered_drive_speed_dist(int fd, __u8 rc_address, __s32 speed_m1, __u32 dist_m1, __s32 speed_m2, __u32 dist_m2, __u8 now) {
	const ssize_t command_size = 20;

	__u8 buffer[command_size] = {
		rc_address,
		BUFFERED_MIX_MODE_DRIVE_SPEED_DIST,

		BYTE(speed_m1, 3),
		BYTE(speed_m1, 2),
		BYTE(speed_m1, 1),
		BYTE(speed_m1, 0),

		BYTE(dist_m1, 3),
		BYTE(dist_m1, 2),
		BYTE(dist_m1, 1),
		BYTE(dist_m1, 0),

		BYTE(speed_m2, 3),
		BYTE(speed_m2, 2),
		BYTE(speed_m2, 1),
		BYTE(speed_m2, 0),

		BYTE(dist_m2, 3),
		BYTE(dist_m2, 2),
		BYTE(dist_m2, 1),
		BYTE(dist_m2, 0),

		now,
		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}


int rc_buffered_m1_drive_speed_accel_dist(int fd, __u8 rc_address, __u32 accel, __s32 speed, __u32 dist, __u8 now){
	const ssize_t command_size = 16;

	__u8 buffer[command_size] = {
		rc_address,
		BUFFERED_M1_DRIVE_SPEED_ACCEL_DIST,

		BYTE(accel, 3),
		BYTE(accel, 2),
		BYTE(accel, 1),
		BYTE(accel, 0),

		BYTE(speed, 3),
		BYTE(speed, 2),
		BYTE(speed, 1),
		BYTE(speed, 0),

		BYTE(dist, 3),
		BYTE(dist, 2),
		BYTE(dist, 1),
		BYTE(dist, 0),

		now,
		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_buffered_m2_drive_speed_accel_dist(int fd, __u8 rc_address, __u32 accel, __s32 speed, __u32 dist, __u8 now){
	const ssize_t command_size = 16;

	__u8 buffer[command_size] = {
		rc_address,
		BUFFERED_M2_DRIVE_SPEED_ACCEL_DIST,

		BYTE(accel, 3),
		BYTE(accel, 2),
		BYTE(accel, 1),
		BYTE(accel, 0),

		BYTE(speed, 3),
		BYTE(speed, 2),
		BYTE(speed, 1),
		BYTE(speed, 0),

		BYTE(dist, 3),
		BYTE(dist, 2),
		BYTE(dist, 1),
		BYTE(dist, 0),

		now,
		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}


int rc_buffered_drive_speed_accel_dist(int fd, __u8 rc_address, __u32 accel, __s32 speed_m1, __u32 dist_m1, __s32 speed_m2, __u32 dist_m2, __u8 now) {
	const ssize_t command_size = 24;

	__u8 buffer[command_size] = {
		rc_address,
		BUFFERED_MIX_MODE_SPEED_ACCEL_DISTANCE,

		BYTE(accel, 3),
		BYTE(accel, 2),
		BYTE(accel, 1),
		BYTE(accel, 0),

		BYTE(speed_m1, 3),
		BYTE(speed_m1, 2),
		BYTE(speed_m1, 1),
		BYTE(speed_m1, 0),

		BYTE(dist_m1, 3),
		BYTE(dist_m1, 2),
		BYTE(dist_m1, 1),
		BYTE(dist_m1, 0),

		BYTE(speed_m2, 3),
		BYTE(speed_m2, 2),
		BYTE(speed_m2, 1),
		BYTE(speed_m2, 0),

		BYTE(dist_m2, 3),
		BYTE(dist_m2, 2),
		BYTE(dist_m2, 1),
		BYTE(dist_m2, 0),

		now,
		0x00 // to be filled with CRC
	};
	fill_crc(buffer, command_size);

	if (rc_uart_write(fd, command_size, buffer) != command_size) {
    	return -1; 
	}
	
    return 0;
}

int rc_read_temperature(int fd, __u8 rc_address, __u16* value) {
    const ssize_t command_size = 2;

    __u8 buffer[4] = {
            rc_address,
            READ_TEMPERATURE};

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
        return -1;
    }

    const ssize_t reply_size = 3;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
        !check_crc(rc_address, READ_TEMPERATURE, in_buffer, reply_size)) {
        return -1;
    }

    *value = (__u16)((in_buffer[0] << 8) | in_buffer[1]);

    return 0;
}

int rc_read_error_status(int fd, __u8 rc_address, __u8* error) {
    const ssize_t command_size = 2;

    __u8 buffer[4] = {
            rc_address,
            READ_ERROR_STATUS};

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
        return -1;
    }

    const ssize_t reply_size = 2;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
        !check_crc(rc_address, READ_ERROR_STATUS, in_buffer, reply_size)) {
        return -1;
    }

    *error = (__u8) in_buffer[0];

    return 0;
}

int rc_read_pid_const_m1(int fd, __u8 rc_address, __u32 *d, __u32 *p, __u32 *i, __u32 *qpps) {
    const ssize_t command_size = 2;

    __u8 buffer[4] = {
            rc_address,
            READ_PID_CONST_M1};

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
        return -1;
    }

    const ssize_t reply_size = 17;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
        !check_crc(rc_address, READ_PID_CONST_M1, in_buffer, reply_size)) {
        return -1;
    }

    *p = (in_buffer[0] << 24) | (in_buffer[1] << 16) | (in_buffer[2] << 8) | in_buffer[3];
    *i = (in_buffer[4] << 24) | (in_buffer[5] << 16) | (in_buffer[6] << 8) | in_buffer[7];
    *d = (in_buffer[8] << 24) | (in_buffer[9] << 16) | (in_buffer[10] << 8) | in_buffer[11];
    *qpps = (in_buffer[12] << 24) | (in_buffer[13] << 16) | (in_buffer[14] << 8) | in_buffer[15];

    return 0;
}

int rc_read_pid_const_m2(int fd, __u8 rc_address, __u32 *d, __u32 *p, __u32 *i, __u32 *qpps) {
    const ssize_t command_size = 2;

    __u8 buffer[4] = {
            rc_address,
            READ_PID_CONST_M2};

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
        return -1;
    }

    const ssize_t reply_size = 17;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
        !check_crc(rc_address, READ_PID_CONST_M2, in_buffer, reply_size)) {
        return -1;
    }

    *p = (in_buffer[0] << 24) | (in_buffer[1] << 16) | (in_buffer[2] << 8) | in_buffer[3];
    *i = (in_buffer[4] << 24) | (in_buffer[5] << 16) | (in_buffer[6] << 8) | in_buffer[7];
    *d = (in_buffer[8] << 24) | (in_buffer[9] << 16) | (in_buffer[10] << 8) | in_buffer[11];
    *qpps = (in_buffer[12] << 24) | (in_buffer[13] << 16) | (in_buffer[14] << 8) | in_buffer[15];

    return 0;
}

int rc_write_to_eeprom(int fd, __u8 rc_address) {
    const ssize_t command_size = 3;

    __u8 buffer[4] = {
            rc_address,
            WRITE_TO_EEPROM,
            0x00 // to be filled with CRC
        };

    fill_crc(buffer, command_size);

    h_uart_flush_input(fd);

    if (rc_uart_write(fd, command_size, buffer) != command_size) {
        return -1;
    }

    for (int i  =0; i < command_size; i++) {
        printf("%x ", buffer[i]);
    }
    printf("\n");

    const ssize_t reply_size = 1;
    __u8 in_buffer[reply_size];
    if (rc_uart_read(fd, reply_size, in_buffer) != reply_size ||
        !check_crc(rc_address, WRITE_TO_EEPROM, in_buffer, reply_size)) {
        return -1;
    }

    return 0;
}