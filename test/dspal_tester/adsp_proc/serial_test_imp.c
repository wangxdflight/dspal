/****************************************************************************
 *   Copyright (c) 2015 James Wilson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <dev_fs_lib_serial.h>
#include <platform.h>
#include <pthread.h>

#include "test_utils.h"
#include "test_status.h"

#include "qc_esc_packet.h"
#include "qc_esc_packet.c"
#include "crc16.h"
#include "crc16.c"

#define SERIAL_TEST_CYCLES 10
#define SERIAL_SIZE_OF_DATA_BUFFER 128
#define SERIAL_WRITE_DELAY_IN_USECS (8000 * 10)

/**
 * Snapdragon Flight DSP supports up to 6 UART devices. However, the actual
 * number of available UART devices may vary depending on the board schematic
 * design. On Snapdragon Flight reference platform, 4 UART ports are available
 * for communication to external serial devices.
 * To make the serial test work for your board, please make sure the following
 * are compliant to your set up:
 * - set NUM_UART_DEVICE_ENABLED with the number of UART ports available
 * - make sure to define UART-BAM mappings in /usr/share/data/adsp/blsp.config
 *   See snapdragon flight user guide for details.
 */
#if defined(DSP_TYPE_ADSP)
#define MAX_UART_DEVICE_NUM      6

const char *serial_device_path[MAX_UART_DEVICE_NUM] = {
	"/dev/tty-1", "/dev/tty-2", "/dev/tty-3",
	"/dev/tty-4", "/dev/tty-5", "/dev/tty-6"
};

#elif defined(DSP_TYPE_SLPI)

#define MAX_UART_DEVICE_NUM      4
const char *serial_device_path[MAX_UART_DEVICE_NUM] = {
	"/dev/tty-5", "/dev/tty-7", "/dev/tty-9",
	"/dev/tty-12" 
};
#endif

int serial_fildes[MAX_UART_DEVICE_NUM] = { -1};
uint8_t tx_data[] = {0xAF, 0x0F, 0x01, 0x51, 0x00, 0x50, 0x00, 0x50, 0x00, 0x50, 0x00, 0xFF, 0x0F, 0x3F, 0xF6}; 


/**
* @brief Test serial device open and close
*
* @par Detailed Description:
* Up to 6 UART devices can be supported. Iterate over /dev/tty-1 to /dev/tty-6
* and open/close each device in O_RDWR mode one by one.
*
* @return
* SUCCESS always
*/
int dspal_tester_serial_multi_port_open(void)
{
	int i;
	int active_devices = 0;
	LOG_INFO("beginning serial device open test");
	int result = SUCCESS;

	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		serial_fildes[i] = open(serial_device_path[i], O_RDWR);
		active_devices += (serial_fildes[i] >= SUCCESS);
		LOG_INFO("open %s O_RDWR mode %s", serial_device_path[i],
			 (serial_fildes[i] < SUCCESS) ? "fail" : "succeed");
	}

	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		if (serial_fildes[i] >= SUCCESS) {
			close(serial_fildes[i]);
		}
	}

	result = (active_devices == NUM_UART_DEVICE_ENABLED) ? SUCCESS : ERROR;

	LOG_INFO("serial multi-port open test %s",
		 result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

void multi_port_read_callback(void *context, char *buffer, size_t num_bytes)
{
	int rx_dev_id = (int)context;
	char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];

	if (num_bytes > 0) {
		memcpy(rx_buffer, buffer, num_bytes);
		rx_buffer[num_bytes] = 0;
    //LOG_INFO("/dev/tty-%d read callback received bytes [%d]",
    //   rx_dev_id, num_bytes);

	} else {
		LOG_ERR("error: read callback with no data in the buffer");
	}
}


/**
* @brief Test multiple serial device at the same time for open,write,read,close.
*
* @par Test:
* 1) Open the serial device /dev/tty-[1-6]. Note: some device open may fail
*    if it is not enabled or configured. See dev_fs_lib_serial.h for more
*    details about how to configure and enable UART devices on the board.
* 2) register read callback on the opened serial devices
* 3) write data to each ports
* 4) close all serial devices
*
* @return
* - SUCCESS if write succeeds on all serial devices through SERIAL_TEST_CYCLES cycles
* - Error otherwise
*/
int dspal_tester_serial_read_callback(void)
{
  int runs;
  int serial_fd = open("/dev/tty-5", O_RDWR);
  
  struct dspal_serial_ioctl_receive_data_callback receive_callback;
  receive_callback.rx_data_callback_func_ptr = multi_port_read_callback;
  receive_callback.context = (void *)(5);

  struct dspal_serial_ioctl_data_rate data_rate;
  data_rate.bit_rate = DSPAL_SIO_BITRATE_250000;

  ioctl(serial_fd, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK, (void *)&receive_callback);
  ioctl(serial_fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&data_rate);
  
  uint8_t tx_data[32];
  
        for (runs = 0; runs < 5000; runs++) {
//                LOG_DEBUG("runs %d", runs);

                int16_t rpm = 5000 + 2*runs;
                
                if (runs<100)  //reset potential snav condition
                  rpm = 0;

                int packet_size = qc_esc_create_rpm_packet4_fb(rpm, rpm, rpm, rpm,
                                     0, 0, 0, 0,
                                     runs%4, tx_data, sizeof(tx_data));
                
                if (packet_size < 1)
                {
                  LOG_ERR("bad packet size %d", packet_size);
                }
                else
                {

                  //uint8_t tx_data[] = {0xAF, 0x0F, 0x01, 0x51, 0x00, 0x50, 0x00, 0x50, 0x00, 0x50, 0x00, 0xFF, 0x0F, 0x3F, 0xF6};
                  //int packet_size = sizeof(tx_data);
                  int num_bytes_written = write(serial_fd, (const char *)tx_data, packet_size);
                  usleep(100);
                  num_bytes_written += write(serial_fd, (const char *)tx_data, packet_size);
                  usleep(100);
                  num_bytes_written += write(serial_fd, (const char *)tx_data, packet_size);
  
                  if (num_bytes_written == 3*packet_size) {
                          LOG_INFO("written %d bytes for i = %d ", num_bytes_written, runs);
                  } else {
                          LOG_ERR("failed to write at %d", runs);
                  }

                }
                usleep(1500);
        }

  close(serial_fd);
  return SUCCESS;
}


int dspal_tester_serial_multi_port_write_read_callback(void)
{
	int result = SUCCESS;
	char tx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	unsigned int num_bytes_written;
	int active_devices = 0;
	int runs, i;

	LOG_INFO("beginning serial multi-port read/write callback test");

	// try to open all uart ports
	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		serial_fildes[i] = open(serial_device_path[i], O_RDWR);
		LOG_INFO("open %s O_RDWR mode %s", serial_device_path[i],
			 (serial_fildes[i] < SUCCESS) ? "fail" : "succeed");
	}

	// set read callback on all uart ports
	struct dspal_serial_ioctl_receive_data_callback receive_callback;
	receive_callback.rx_data_callback_func_ptr = multi_port_read_callback;

	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		if (serial_fildes[i] >= SUCCESS) {
			receive_callback.context = (void *)(i + 1);

			result = ioctl(serial_fildes[i],
				       SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,
				       (void *)&receive_callback);
			LOG_INFO("set serial read callback on %s %s",
				 serial_device_path[i], result < SUCCESS ? "failed" : "succeeded");

			if (result < SUCCESS) {
				close(serial_fildes[i]);
				serial_fildes[i] = -1;
			}
		}
	}

	for (runs = 0; runs < SERIAL_TEST_CYCLES; runs++) {
		LOG_DEBUG("runs %d", runs);

		active_devices = 0;

		for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
			if (serial_fildes[i] < SUCCESS) {
				continue;
			}

			memset(tx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
			sprintf(tx_buffer, "message from /dev/tty-%d\n", i + 1);

			num_bytes_written = write(serial_fildes[i],
						  (const char *)tx_buffer,
						  strlen(tx_buffer));

			if (num_bytes_written == strlen(tx_buffer)) {
				LOG_DEBUG("written %d bytes to %s", num_bytes_written,
					  serial_device_path[i]);
				active_devices++;

			} else {
				LOG_ERR("failed to write to %s", serial_device_path[i]);
				close(serial_fildes[i]);
				serial_fildes[i] = -1;
			}
		}

		if (active_devices == 0) {
			break;
		}

		usleep(SERIAL_WRITE_DELAY_IN_USECS);
	}

	// close all devices
	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		if (serial_fildes[i] >= SUCCESS) {
			close(serial_fildes[i]);
		}
	}

	if (!(runs == SERIAL_TEST_CYCLES && active_devices == NUM_UART_DEVICE_ENABLED)) {
		result = ERROR;
	}

	LOG_INFO("serial multi-port read/write callback test %s",
		 result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test serial read and write functionality
*
* @par Detailed Description:
* The serial bus has its RX and TX wired together so it can do a loop-back of the data.
* Data is sent over the bus and read back using a read call
*
* Test:
* 1) Open the serial device /dev/tty-[1-6]
* 2) Write data to the serial device
* 3) Read data using read call, print out received data if available
* 4) Loop steps 2-3 for SERIAL_TEST_CYCLES number of loops
* 5) Close all serial devices
*
* @return
* - SUCCESS if write succeeds on all serial devices through SERIAL_TEST_CYCLES cycles
* - Error otherwise
*/
int dspal_tester_serial_multi_port_write_read(void)
{
	int result = SUCCESS;
	unsigned int num_bytes_written = 0;
	int num_bytes_read = 0;
	char tx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	int active_devices;
	int runs, i;

	LOG_INFO("beginning multi-port serial read/write test");

	// try to open all uart ports
	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		serial_fildes[i] = open(serial_device_path[i], O_RDWR);
		LOG_INFO("open %s O_RDWR mode %s", serial_device_path[i],
			 (serial_fildes[i] < SUCCESS) ? "fail" : "succeed");
	}

	// repeatedly write and read from each opened serial port
	for (runs = 0; runs < SERIAL_TEST_CYCLES; runs++) {
		LOG_DEBUG("runs %d", runs);
		active_devices = 0;

		for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
			if (serial_fildes[i] < SUCCESS) {
				continue;
			}

			memset(tx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
			sprintf(tx_buffer, "message from /dev/tty-%d\n", i + 1);

			num_bytes_written = write(serial_fildes[i],
						  (const char *)tx_buffer,
						  strlen(tx_buffer));

			if (num_bytes_written == strlen(tx_buffer)) {
				LOG_DEBUG("written %d bytes to %s", num_bytes_written,
					  serial_device_path[i]);
				active_devices++;

			} else {
				LOG_ERR("failed to write to %s", serial_device_path[i]);
				close(serial_fildes[i]);
				serial_fildes[i] = -1;
				continue;
			}

			memset(rx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
			num_bytes_read = read(serial_fildes[i], rx_buffer,
					      SERIAL_SIZE_OF_DATA_BUFFER);
			LOG_DEBUG("%s read bytes [%d]: %s",
				  serial_device_path[i], num_bytes_read, rx_buffer);
		}

		if (active_devices == 0) {
			break;
		}

		usleep(SERIAL_WRITE_DELAY_IN_USECS);
	}

	// close all devices
	for (i = 0; i < NUM_UART_DEVICE_ENABLED; i++) {
		if (serial_fildes[i] >= SUCCESS) {
			close(serial_fildes[i]);
		}
	}

	if (!(runs == SERIAL_TEST_CYCLES && active_devices == NUM_UART_DEVICE_ENABLED)) {
		result = ERROR;
	}

	LOG_INFO("serial multi-port read/write test %s",
		 result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

/**
* @brief Test serial read with small buffer
*
* @par Detailed Description:
* This test case is testing the scenario in which the serial bus receives X
* bytes. User calls read() to read the data and passes in a small buffer.
* read() should return -EINVAL error code in this caes
*
* Test:
* 1) Open the serial device /dev/tty-1
* 2) Write very long bytes to the serial device
* 3) wait for 100ms to make sure the loopback data is received
* 3) read() with 10 byte buffer and check the return value
* 5) Close serial device
*
* @return
* - SUCCESS
*/
int dspal_tester_serial_read_with_small_buffer(void)
{
	int result = SUCCESS;
	int num_bytes_written = 0;
	int num_bytes_read = 0;
	char tx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	int fd;
	int max_read_bytes = 20;
	int devid = 1;

	LOG_INFO("beginning serial read with small buffer test");

	fd = open(serial_device_path[devid], O_RDWR);
	LOG_INFO("open %s O_RDWR mode %s", serial_device_path[devid],
		 (fd < SUCCESS) ? "fail" : "succeed");

	if (fd < SUCCESS) {
		result = ERROR;
		goto exit;
	}

	memset(tx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
	sprintf(tx_buffer, "message from /dev/tty-%d\n", devid);

	num_bytes_written = write(fd,
				  (const char *)tx_buffer,
				  SERIAL_SIZE_OF_DATA_BUFFER);

	if (num_bytes_written == SERIAL_SIZE_OF_DATA_BUFFER) {
		LOG_DEBUG("written %d bytes to %s", num_bytes_written,
			  serial_device_path[devid]);

	} else {
		LOG_ERR("failed to write to %s", serial_device_path[devid]);
		goto exit;
	}

	// wait 100ms to ensure the data is received in the loopback
	usleep(100000);
	memset(rx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
	num_bytes_read = read(fd, rx_buffer, max_read_bytes);

	if (num_bytes_read == -1) {
		LOG_DEBUG("%s read() with small buffer return expected error code -1",
			  serial_device_path[devid]);

	} else {
		LOG_ERR("%s read() return: %d, expected -1",
			serial_device_path[devid], num_bytes_read);
	}

exit:

	if (fd >= SUCCESS) {
		close(fd);
	}

	LOG_INFO("serial read with small buffer test %s",
		 result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}

#define PACKET_INTERVAL_IN_USECS 2000  // 2ms

int64_t get_time_1us(void)
{
   uint64_t timestamp = 0;

   struct timespec ts = { 0, 0 };

   clock_gettime(CLOCK_MONOTONIC, &ts);

   timestamp += ts.tv_sec * 1000000;
   timestamp += ts.tv_nsec / 1000;

   return timestamp;
}

void* dspal_tester_serial_write(void* esc_fd)
{
    int fd = (int)esc_fd; 
    int repeat_cnt = 3;
    int64_t last_packet_time_in_usecs = 0;
    /*Spin motor*/
    //uint8_t out[] = {0xAF, 0x0F, 0x01, 0x51, 0x00, 0x50, 0x00, 0x50, 0x00, 0x50, 0x00, 0xFF, 0x0F, 0x3F, 0xF6}; 
    int packet_size = sizeof(tx_data); 

    for (int i = 0; i < 50000; i++) 
    {
        last_packet_time_in_usecs = get_time_1us();
        for (int j = 0; j < repeat_cnt; j ++) 
        {
            int num_bytes_written = write(fd,(const char *)tx_data,packet_size);

            if (num_bytes_written == packet_size) {
                LOG_INFO("written %d bytes for i = %d ", num_bytes_written,i);
            } else {
                LOG_ERR("failed to write");
                return NULL; 
            }   
            usleep(100);
        }
        int sleep_time = (int) (PACKET_INTERVAL_IN_USECS - (get_time_1us() - last_packet_time_in_usecs));
        LOG_ERR("to sleep %d", sleep_time);
        usleep(sleep_time); 
    }

#if 0 
    /* Assign IDs: 
      Front Left 0:  0xAF, 0x06, 0x0B, 0x00, 0x96, 0xF1
      Rear Left 1:   0xAF, 0x06, 0x0B, 0x01, 0x57, 0x31
      Rear Right 2:  0xAF, 0x06, 0x0B, 0x02, 0x17, 0x30
      Front Right 3: 0xAF, 0x06, 0x0B, 0x03, 0xD6, 0xF0 
    */ 

    uint8_t id_assign[] = {0xAF, 0x06, 0x0B, 0x01, 0x57, 0x31}; 
    int packet_size = sizeof(id_assign); 

    int num_bytes_written = write(fd,(const char *)id_assign,packet_size);

    if (num_bytes_written == packet_size) {
        LOG_INFO("written %d bytes ", num_bytes_written);

    } else {
        LOG_ERR("failed to write");
        return NULL; 
    }
    usleep(100); 
#endif

    return NULL; 
}
void dspal_tester_serial_read_callback_(void *context, char *buffer, size_t num_bytes)
{
	int rx_dev_id = (int)context;
	char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];

	if ((buffer != NULL ) && (num_bytes > 0)) {
		memcpy(rx_buffer, buffer, num_bytes);
		rx_buffer[0] = 0;
		LOG_ERR("/dev/tty-%d read callback received bytes: %d",
			 rx_dev_id, num_bytes);

        for (int i = 0; i < (int)num_bytes; i++) 
        {
            //LOG_ERR("buffer[%d]=%x", i , buffer[i]);
        }
		if (num_bytes != 12)
			LOG_ERR("!!!!!!!!!!!!!!!!!!available bytes %d", num_bytes);

	} else {
		LOG_ERR("error: read callback with no data in the buffer");
	}
}

int dspal_tester_serial_open_write(void)
{
    LOG_INFO("beginning serial device write test");
	int result = SUCCESS;

    int esc_fd = open("/dev/tty-5", O_RDWR);
    LOG_INFO("open /dev/tty-5 O_RDWR mode %s", 
             (esc_fd < SUCCESS) ? "fail" : "succeed");
	
    if (esc_fd < SUCCESS) {
        result = ERROR; 
        goto exit;
    }
#if 0	
	struct dspal_serial_ioctl_receive_data_callback receive_callback;
	receive_callback.rx_data_callback_func_ptr = dspal_tester_serial_read_callback_;
	receive_callback.context = (void *)(5);
	result = ioctl(esc_fd, 
		       SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,
		       (void *)&receive_callback);
	if (result < SUCCESS) {
		close(esc_fd);
		esc_fd = -1;
	}
#endif
	struct dspal_serial_ioctl_data_rate data_rate;
	data_rate.bit_rate = DSPAL_SIO_BITRATE_250000;

	ioctl(esc_fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&data_rate);

    pthread_t thread;
    pthread_attr_t attr;
    size_t stacksize = 2 * 1024;

    int rv = pthread_attr_init(&attr);

    if (rv != 0) { FAIL("pthread_attr_init returned error"); }

    rv = pthread_attr_setstacksize(&attr, stacksize);

    if (rv != 0) { FAIL("pthread_attr_setstacksize returned error"); }

    /*
     * Create the thread passing a reference to the cond structure
     * just initialized.
     */
    rv = pthread_create(&thread, &attr, dspal_tester_serial_write, (void*)esc_fd);

    if (rv != 0) {
        LOG_ERR("error pthread_create: %d", rv);
        goto exit;
    }
    rv = pthread_join(thread, NULL);

	if (rv != 0) {
		LOG_ERR("error pthread_join: %d", rv);
		goto exit;
	}
        
	close(esc_fd);
	
	
exit:
	LOG_INFO("serial open/ESC motor spin test %s",
		 result == SUCCESS ? "PASSED" : "FAILED");

	return result;
}


/**
* @brief Runs all the serial tests and returns 1 aggregated result.
*
* @return
* SUCCESS ------ All tests pass
* ERROR -------- One or more tests failed
*/
int dspal_tester_serial_test(void)
{
	int result;
    //result = dspal_tester_serial_open_write();
    result = dspal_tester_serial_read_callback();
	if (result < SUCCESS) {
		return result;
	}
#if defined(DSP_TYPE_ADSP)
	// serial devices open test
	result = dspal_tester_serial_multi_port_open();

	if (result < SUCCESS) {
		return result;
	}

	// multi-port write/read test with rx callback
	result = dspal_tester_serial_multi_port_write_read_callback();

	if (result < SUCCESS) {
		return result;
	}

	// multi-port read/write test
	result = dspal_tester_serial_multi_port_write_read();

	if (result < SUCCESS) {
		return result;
	}

	result = dspal_tester_serial_read_with_small_buffer();

	if (result < SUCCESS) {
		return result;
	}
#endif
	return SUCCESS;
}
