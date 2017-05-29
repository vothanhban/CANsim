/*
 * linux_uart.c
 *
 *  Created on: Nov 27, 2014
 *      Author: nvthanh
 */



#include "LinuxUart.h"
#include "Debug.h"
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#  define CCLAIMED 0x80000000

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

linux_uart_data *	linux_uart_open (uint8_t * port)
{
	struct linux_uart_data * ret=malloc(sizeof(struct linux_uart_data));
	if(!ret){
		FATAL("Out of memory !!!\n");
		return NULL;
	}
	ret->fd =open((char*)port, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(ret->fd <0 ){
		FATAL("Cannot open uart port: %s\n",port);
		linux_uart_close(ret);
		return NULL;
	}
	struct termios term;

	if (tcgetattr(ret->fd, &term) == -1) {
		linux_uart_close(ret);
		return NULL;
	}
	// Make sure the port is not claimed already
	if (term.c_iflag & CCLAIMED) {
		linux_uart_close(ret);
		return NULL;;
	}
	// Copy the old terminal info struct

	term.c_cflag = CS8 | CLOCAL | CREAD;
	term.c_iflag = CCLAIMED | IGNPAR;
	term.c_oflag = 0;
	term.c_lflag = 0;

	term.c_cc[VMIN] = 0;     // block until n bytes are received
	term.c_cc[VTIME] = 0;    // block until a timer expires (n * 100 mSec.)

	if (tcsetattr(ret->fd, TCSANOW, &term) == -1) {
		linux_uart_close(ret);
		return NULL;
	}
	return ret;
}

int 	linux_uart_close (struct linux_uart_data * pud)
{
	if(!pud) return 1;
	if(pud->fd>0) close(pud->fd);
	free(pud);
	return 0;
}
#if 0
int 	linux_uart_flush (struct linux_uart_data * pud)
{
	if(!pud || (pud->fd <= 0) ) return false;
	int res;
	res = tcflush(pud->fd, TCIOFLUSH);

	// So, I wrote this byte-eater
	// Retrieve the count of the incoming bytes
//	printf("================== %s %d %d\n",__FUNCTION__,__LINE__,res);
	int available_bytes_count = 0;
	fd_set rfds;


	// Reset file descriptor
	FD_ZERO(&rfds);
	FD_SET(pud->fd, &rfds);

	struct timeval timeout_tv;
	timeout_tv.tv_sec = 0;
	timeout_tv.tv_usec = 2000/* + 2000*/;
	res = select(pud->fd + 1, &rfds, NULL, NULL,  &timeout_tv);
	if ((res < 0) && (EINTR == errno)) {
		return 0;
	}

	res = ioctl(pud->fd, FIONREAD, &available_bytes_count);
	if (res != 0) {
		return 0;
	}

//	printf("%s %d %d\n",__FUNCTION__,__LINE__,available_bytes_count);
	if (available_bytes_count == 0) {
		return 0;
	}
	char *rx = malloc(available_bytes_count);
	if (!rx) {
		perror("malloc");
		return 1;
	}
	// There is something available, read the data
	(void)read(pud->fd, rx, available_bytes_count);
	free(rx);
	return 0;
}
#else

int 	linux_uart_flush (struct linux_uart_data * pud)
{
	if(!pud || (pud->fd <= 0) ) return false;
	tcflush(pud->fd, TCIFLUSH);
	// So, I wrote this byte-eater
	// Retrieve the count of the incoming bytes
	int available_bytes_count = 0;
	int res;
	res = ioctl(pud->fd, FIONREAD, &available_bytes_count);
	if (res != 0) {
		return 0;
	}
	if (available_bytes_count == 0) {
		return 0;
	}
	char *rx = malloc(available_bytes_count);
	if (!rx) {
		perror("malloc");
		return 1;
	}
	// There is something available, read the data
	(void)read(pud->fd, rx, available_bytes_count);
	free(rx);
	return 0;
}
#endif

size_t	linux_uart_write (struct linux_uart_data * pud,uint8_t * data,size_t szdata)
{
	if(!pud || (pud->fd <= 0) ) return 0;
	return write(pud->fd, data, szdata);
}

size_t	 linux_uart_read (struct linux_uart_data * pud,uint8_t * resdata,size_t szexp,uint32_t timeout)
{

	int received_bytes_count = 0;
	int available_bytes_count = 0;
	const int expected_bytes_count = (int)szexp;
	fd_set rfds;
	int res;
	size_t retLen;
	do {
		select:
		// Reset file descriptor
		FD_ZERO(&rfds);
		FD_SET(pud->fd, &rfds);
		struct timeval timeout_tv;
		if (timeout > 0) {
			timeout_tv.tv_sec = (timeout / 1000) ;
			timeout_tv.tv_usec = ((timeout % 1000) * 1000) /*+ 2000*/;
		}else{
			timeout_tv.tv_sec = 0;
			timeout_tv.tv_usec = 200/* + 2000*/;
		}
		res = select(pud->fd + 1, &rfds, NULL, NULL,  &timeout_tv);
		if ((res < 0) && (EINTR == errno)) {
			// The system call was interupted by a signal and a signal handler was
			// run.  Restart the interupted system call.
			goto select;
		}

		// Read error
		if (res <= 0) {
			return received_bytes_count;
		}
		// Retrieve the count of the incoming bytes
		res = ioctl(pud->fd, FIONREAD, &available_bytes_count);
		if (res != 0) {
			return received_bytes_count;
		}
		// There is something available, read the data
		res = read(pud->fd, resdata + received_bytes_count, MIN(available_bytes_count, (expected_bytes_count - received_bytes_count)));
		// Stop if the OS has some troubles reading the data
		if (res <= 0) {
			return received_bytes_count;
		}
		received_bytes_count += res;
		retLen = received_bytes_count;
	} while (expected_bytes_count > received_bytes_count);
	return retLen;
}

int linux_uart_set_speed (struct linux_uart_data * pud,uint32_t baudrate)
{
	speed_t stPortSpeed = B9600;
	switch (baudrate) {
	case 9600:
		stPortSpeed = B9600;
		break;
	case 19200:
		stPortSpeed = B19200;
		break;
	case 38400:
		stPortSpeed = B38400;
		break;
#  ifdef B57600
	case 57600:
		stPortSpeed = B57600;
		break;
#  endif
#  ifdef B115200
	case 115200:
		DBG("set speed to 115200\n");
		stPortSpeed = B115200;
		break;
#  endif
#  ifdef B230400
	case 230400:
		stPortSpeed = B230400;
		break;
#  endif
#  ifdef B460800
	case 460800:
		stPortSpeed = B460800;
		break;
#  endif
	default:
		return 1;
	};
	struct termios term;

	if (tcgetattr(pud->fd, &term) == -1) {
		FATAL("\n");
		return false;
	}
	// Set port speed (Input and Output)
	cfsetispeed(&term, stPortSpeed);
	cfsetospeed(&term, stPortSpeed);
	if (tcsetattr(pud->fd, TCSADRAIN, &term) == -1) {
		FATAL("\n");
		return false;
	}
	return 0;
}
