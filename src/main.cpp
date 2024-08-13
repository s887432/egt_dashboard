/*
 * Copyright (C) 2018 Microchip Technology Inc.  All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <egt/ui>
#include <memory>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <random>
#include <chrono>
#include <queue>
#include <poll.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/stat.h>
#include "eraw.h"

#define ENABLE_UART
#define ENABLE_CAN


// need to modify manually
#define BACKGROUND_OFFSET	2
#define NAVIGATION_LEFT		0
#define NAVIGATION_RIGHT	1

// ********************************************************************************************************
#ifdef ENABLE_CAN
// ********************************************************************************************************
#include <linux/can.h>
#include <linux/can/raw.h>

static int canInit(char *canName);
// ********************************************************************************************************
#endif // end of ENABLE_CAN
// ********************************************************************************************************


// ********************************************************************************************************
#ifdef ENABLE_UART
// ********************************************************************************************************
int speed_arr[] = { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300,
		B38400, B19200, B9600, B4800, B2400, B1200, B300, };

int name_arr[] = {115200, 57600, 38400,  19200,  9600,  4800,  2400,  1200,  300,
		38400,  19200,  9600, 4800, 2400, 1200,  300, };
		
static int uartOpen(char *Dev);
static void uartSetSpeed(int fd, int speed);
static int uartSetParity(int fd,int databits,int stopbits,int parity);	

// ********************************************************************************************************
#define RECEIVE_BUFFER_SIZE	512

#define CMD_HEADER_1	0xEF
#define CMD_HEADER_2	0x12

#define HEADER_OFFSET	0
#define HEADER_SIZE		2

#define COMMAND_OFFSET	2
#define COMMAND_SIZE	1

#define LENGTH_OFFSET	3
#define LENGTH_SIZE		1

#define PAYLOAD_OFFSET	4

typedef enum __BLE_COMMANDS__
{
	COMMAND_NONE = 0,
	COMMAND_DRIVE_INTO,
	COMMAND_ESTIMATE_MILES,
	COMMAND_NAVI_DIRECTION,
	COMMAND_CALLING_INFO,
	COMMAND_MSG_KEY,
}BLE_COMMANDS;

typedef enum __NAVI_DIRECTIONS_
{
	NAVIDIR_ARRIVED=0,
	NAVIDIR_LOWER_LEFT,
	NAVIDIR_LEFT,
	NAVIDIR_UPPER_LEFT,
	NAVIDIR_STRAIGHT,
	NAVIDIR_UPPER_RIGHT,
	NAVIDIR_RIGHT,
	NAVIDIR_LOWER_RIGHT,
	NAVIDIR_LEFT_U_TURN,
	NAVIDIR_RIGHT_U_TURN,
}NAVI_DIRECTIONS;

typedef struct __NAVI_STR__
{
	int distance;
	NAVI_DIRECTIONS direction;
	// TODO... name
}NaviStr, *pNaviStr;	

static int bleCheckCommand(unsigned char *recBuf, int length, unsigned char *cmdBuf);
static BLE_COMMANDS bleGetCommand(unsigned char *cmdBuf);
static int bleNaviReconstruct(unsigned char *cmdBuf, pNaviStr navi);
// ********************************************************************************************************
#endif // end of ENABLE_UART
// ********************************************************************************************************

typedef unsigned char u8;
size_t getFileSize(const char *fileName);

class MotorDash : public egt::experimental::SVGDeserial
{
public:

    explicit MotorDash(egt::TopWindow& parent)
        : egt::experimental::SVGDeserial(parent)
    {
    }

private:
};


size_t getFileSize(const char *fileName) {
	if (fileName == NULL) {
		return 0;
	}
	
	struct stat statbuf;
	stat(fileName, &statbuf);
	size_t filesize = statbuf.st_size;

	return filesize;
}

int main(int argc, char** argv)
{
	std::shared_ptr<egt::experimental::GaugeLayer> naviLeft;
	std::shared_ptr<egt::experimental::GaugeLayer> naviRight;
	
	bool isUpdated = false;
	
#ifdef ENABLE_CAN
	int canFd = -1;
	int nCanReadBytes;
	struct can_frame frame;
	struct pollfd pollCanfds;
	struct pollfd pollUartfds;
	
	canFd = canInit((char*)"vcan0");
	if( canFd < 0 )
	{
		std::cout << "Initial can error" << std::endl;
	}
	
	pollCanfds.fd = canFd;
	pollCanfds.events = POLLRDNORM;
#endif

#ifdef ENABLE_UART
	unsigned char recBuff[RECEIVE_BUFFER_SIZE];
	unsigned char cmdBuff[RECEIVE_BUFFER_SIZE];	
	unsigned char receiveBuffer[RECEIVE_BUFFER_SIZE];
	int nUartRead;
	int receiveIndex = 0;
	int cmdSize;
	
	NaviStr naviInfo;
	BLE_COMMANDS cmd;
	
	if( argc != 2 )
	{
		std::cout << "USAGE: uart_transmit UART_PORT" << std::endl;
		return -1;
	}
	
	int fdUart;
	fdUart = uartOpen(argv[1]);
	uartSetSpeed(fdUart, 115200);

	if (uartSetParity(fdUart,8,1,'N') == -1)
	{
		printf("Set Parity Error\n");
		return -1;
	}
	else
	{
		printf("%s connected\r\n", argv[1]);
	}
	
	pollUartfds.fd = fdUart;
	pollUartfds.events = POLLRDNORM;
#endif
	
    std::cout << "EGT start" << std::endl;

    egt::Application app(argc, argv);  //This call will cost ~270ms on 9x60ek board
    egt::TopWindow window;
    
    window.color(egt::Palette::ColorId::bg, egt::Palette::black);

    MotorDash motordash(window);

    window.show();
    
    // Read eraw.bin to buffer
    const char* ERAW_NAME = "eraw.bin";
    std::string erawname;
    size_t buff_size = getFileSize(ERAW_NAME);
    void* buff_ptr = NULL;
    if (buff_size) {
        buff_ptr = malloc(buff_size);
    } else {
        std::cout << "eraw.bin is blank" << std::endl;
        return -1;
    }

    std::ifstream f(ERAW_NAME, std::ios::binary);
	if(!f)
	{
		std::cout << "read eraw.bin failed" << std::endl;
        free(buff_ptr);
		return -1;
	}
	f.read((char*)buff_ptr, buff_size);

    std::cout << "EGT show" << std::endl;

    //Lambda for de-serializing background and needles
    auto DeserialNeedles = [&]()
    {
        //Background image and needles should be de-serialized firstly before main() return
        motordash.AddWidgetByBuf((const u8*)buff_ptr+offset_table[BACKGROUND_OFFSET].offset, offset_table[BACKGROUND_OFFSET].len, true);
        
        naviLeft = motordash.AddWidgetByBuf((const u8*)buff_ptr+offset_table[NAVIGATION_LEFT].offset, offset_table[NAVIGATION_LEFT].len, true);
        naviRight = motordash.AddWidgetByBuf((const u8*)buff_ptr+offset_table[NAVIGATION_RIGHT].offset, offset_table[NAVIGATION_RIGHT].len, true);
        
        naviLeft->move(egt::Point(136, 221));
        naviRight->move(egt::Point(136, 221));
        naviLeft->hide();
        naviRight->hide();
    };

    DeserialNeedles();
    
    int count=1;
    // 
	egt::PeriodicTimer timer(std::chrono::milliseconds(10));
	timer.on_timeout([&]()
    {
#ifdef ENABLE_CAN
		if( 0 < poll(&pollCanfds, 1, 0) )
		{
			// check if any data from CAN
			if( (nCanReadBytes = read(canFd, &frame, sizeof(struct can_frame))) > 0 )
			{
				// set display flag only. don't update image in this loop
				// TODO...
				for(int i=0; i<nCanReadBytes; i++)
				{
					printf("%02X ", frame.data[i]);
				}
			}
		}
#endif
    
#ifdef ENABLE_UART    
		if( 0 < poll(&pollUartfds, 1, 0) )
		{
			// check if any data came from UART
			if( (nUartRead = read(fdUart, recBuff, 512)) >0)
			{
				if( nUartRead+receiveIndex >= RECEIVE_BUFFER_SIZE )
				{
					std::cout << "receiver buffer overflow. it must be something wrong." << std::endl;
					// TODO...
					receiveIndex = 0;
				} 

				// append received data to working buffer
				memcpy(receiveBuffer+receiveIndex, recBuff, nUartRead); 
				receiveIndex += nUartRead;
				
				// retrieve command package
				cmdSize = bleCheckCommand(recBuff, receiveIndex, cmdBuff);
				
				// set display flag only. don't update image in this loop
				if( cmdSize > 0 )
				{
					cmd = bleGetCommand(cmdBuff);
					printf("Command=%d\r\n", cmd);
					switch( cmd )
					{
						case COMMAND_NAVI_DIRECTION:
							bleNaviReconstruct(cmdBuff, &naviInfo);
							printf("distance=%d, direction=%d\r\n", naviInfo.distance, naviInfo.direction);
							switch( naviInfo.direction )
							{
								case NAVIDIR_LEFT:
									naviLeft->show();
									isUpdated = true;
									break;
								case NAVIDIR_RIGHT:
									naviRight->show();
									isUpdated = true;
									break;
								default:
									break;
							}
							break;
							
						default:
							printf("Uknown command (%d)\r\n", cmd);
							break;
					}
				}
			}
    	}
#endif // end of ENABLE_UART    	
    	
    	count++;
		if( count >= 300)
		{
			//std::cout<<"timeout"<<std::endl;
			count=1;
			
			if( isUpdated )
			{
				if( naviLeft->visible() )
				{
					naviLeft->hide();
				}
				
				if( naviRight->visible() )
				{
					naviRight->hide();
				}
			
				isUpdated = false;
			}
		}
    });
    timer.start();
     
    return app.run();
}

#ifdef ENABLE_UART
// ********************************************************************************************************
// uart process
// ********************************************************************************************************
static int uartOpen(char *Dev)
{
	int fd = open( Dev, O_RDWR );
	struct termios  tty;
	int rc;

	if (-1 == fd)   
	{
		perror("Can't Open Serial Port");
		return -1;              
	}

	rc = tcgetattr(fd, &tty);
    if (rc < 0) {
        printf("failed to get attr: %d, %s\r\n", rc, strerror(errno));
        return -1;
    }

    cfmakeraw(&tty);

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 10;

    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;    /* no HW flow control? */
    tty.c_cflag |= CLOCAL | CREAD;
    rc = tcsetattr(fd, TCSANOW, &tty);
    if (rc < 0) {
        printf("failed to set attr: %d, %s\r\n", rc, strerror(errno));
        return -1;
    }

	return fd;
}

static void uartSetSpeed(int fd, int speed)
{
	int   i;
	int   status;
	struct termios   Opt;
	
	tcgetattr(fd, &Opt);

	for ( i= 0;  i < (int)(sizeof(speed_arr) / sizeof(int));  i++)
	{
		if  (speed == name_arr[i])
		{
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &Opt);
	
			if  (status != 0)
			{
				perror("tcsetattr fd1");
			}

			return;
		}

		tcflush(fd,TCIOFLUSH);
	}
}

static int uartSetParity(int fd,int databits,int stopbits,int parity)
{
	struct termios options;

	if  ( tcgetattr( fd,&options)  !=  0)
	{
		perror("SetupSerial 1");
		return(-1);
	}

	options.c_cflag &= ~CSIZE;

	switch (databits)
	{
		case 7:
			options.c_cflag |= CS7;
			break;

		case 8:
			options.c_cflag |= CS8;
			break;

		default:
			fprintf(stderr,"Unsupported data size\n");
			return (-1);
	}

	switch (parity)
	{
		case 'n':
		case 'N':
			options.c_cflag &= ~PARENB;
			options.c_iflag &= ~INPCK;
			break;

		case 'o':
		case 'O':
			options.c_cflag |= (PARODD | PARENB);
			options.c_iflag |= INPCK;
			break;

		case 'e':
		case 'E':
			options.c_cflag |= PARENB;
			options.c_cflag &= ~PARODD;
			options.c_iflag |= INPCK;
			break;

		case 'S':
		case 's':
			options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;
			break;

		default:
			fprintf(stderr,"Unsupported parity\n");
			return (-1);
	}

	switch (stopbits)
	{
		case 1:
			options.c_cflag &= ~CSTOPB;
			break;

		case 2:
			options.c_cflag |= CSTOPB;
			break;

		default:
			fprintf(stderr,"Unsupported stop bits\n");
			return (-1);
	}

	if (parity != 'n')
	{
		options.c_iflag |= INPCK;
	}

	options.c_cc[VTIME] = 150;
    options.c_cc[VMIN] = 0;

	tcflush(fd,TCIFLUSH);

	if (tcsetattr(fd,TCSANOW,&options) != 0)
	{
		perror("SetupSerial 3");
		return -1;
	}

	return (0);
}

// re-position the buffer if 0xEF 0x12 is not in beginning of buffer
static int bleCheckCommand(unsigned char *recBuf, int length, unsigned char *cmdBuf)
{
	int cmdSize = 0;
	int header = -1;
	int tail = 0;
	int index = 0;

	int package_size = 0;

	// find header
	for(int i=0; i<length-1; i++)
	{
		if( recBuf[i] == CMD_HEADER_1 && recBuf[i+1] == CMD_HEADER_2 )
		{
			header = i;
			index = i+HEADER_SIZE;
			break;
		}
	}

	if( header >= 0 )
	{
		// header found
		if( (index + COMMAND_SIZE) <= length )
		{
			// command existed
			index += COMMAND_SIZE;
		}
		else
		{
			// command package is not complete
			return cmdSize;
		}

		// command found
		if( (index + LENGTH_SIZE) <= length )
		{
			// length existed
			package_size = recBuf[index];
			index += COMMAND_SIZE;			
		}
		else
		{
			// command package is not complete
			return cmdSize;
		}

		// package size found
		if( (index + package_size) <= length )
		{
			// payload existed
			index += package_size;
		}
		else
		{
			// command package is not complete
			return cmdSize;
		}

		// payload found
		if( (index + 2) <= length )
		{
			// tail existed
			index += 2;
			tail = index -1;
		}
		else
		{
			// command package is not complete
			return cmdSize;
		}

		// command is complete
		memcpy(cmdBuf, recBuf+header, index);
		cmdSize = tail - header;

		// move buffer
		if( length >= index )
		{
			memcpy(recBuf, recBuf+index, length-index-2);
		}
	}

	if( header >= 0 && tail >=0 )
	{
		// command package is complete
		
	}

	return cmdSize;
}

static BLE_COMMANDS bleGetCommand(unsigned char *cmdBuf)
{
	if( cmdBuf[HEADER_OFFSET] != CMD_HEADER_1 || cmdBuf[HEADER_OFFSET+1] != CMD_HEADER_2 )
	{
		return COMMAND_NONE;
	}
	
	return (BLE_COMMANDS)cmdBuf[COMMAND_OFFSET];
}

static int bleNaviReconstruct(unsigned char *cmdBuf, pNaviStr navi)
{
	if( cmdBuf[HEADER_OFFSET] != CMD_HEADER_1 || cmdBuf[HEADER_OFFSET+1] != CMD_HEADER_2 )
	{
		return -2;
	}
	
	if( (cmdBuf[COMMAND_OFFSET] != COMMAND_NAVI_DIRECTION) )
	{
		return -1;
	}
	
	navi->distance = cmdBuf[PAYLOAD_OFFSET]*256*256 + cmdBuf[PAYLOAD_OFFSET+1]*256 + cmdBuf[PAYLOAD_OFFSET+2];
	navi->direction = (NAVI_DIRECTIONS)cmdBuf[PAYLOAD_OFFSET+3];
	
	return 0;
}
#endif	// end of ENABLE_UART

#ifdef ENABLE_CAN
// ********************************************************************************************************
// can process
// ********************************************************************************************************

static int canInit(char *canName)
{
	int canFd = -1;
	struct sockaddr_can addr;
//	struct ifreq ifr;
	
	if ((canFd = socket(AF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return -1;
	}

//	strcpy(ifr.ifr_name, canName);
//	ioctl(canFd, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = 0;//ifr.ifr_ifindex;

	if (bind(canFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return -1;
	}
		
	return canFd;
}

// ********************************************************************************************************
#endif // end of ENABLE_CAN

// end of file
