/**
 * Raspberry - CameraVC0706 implementation.
 * 
 * CameraVC0706.h
 * 
 * The class CameraVC0706.
 * 
 * @author Dalmir da Silva <dalmirdasilva@gmail.com>
 */

#ifndef __RASPBERRY_DRIVER_CAMERA_VC0706_H__
#define __RASPBERRY_DRIVER_CAMERA_VC0706_H__ 1

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>

#define VC0760_DEBUG 				1
#define VC0760_PROTOCOL_SIGN_TX     0x56
#define VC0760_PROTOCOL_SIGN_RX     0x76

#define VC0760_RX_BUFFER_SIZE       0x10
#define VC0760_CAMERA_DELAY 		0x0100

class CameraVC0706 {

	int fd;

	char dev[100];

	unsigned char rxBuffer[VC0760_RX_BUFFER_SIZE];

	int rxBufferPointer;

	unsigned char serialNumber;

	int framePointer;

public:

	enum Command {

		// Get Firmware version information
		GEN_VERSION = 0x11,

		// Set serial number
		SET_SERIAL_NUMBER = 0x21,

		// Set port
		SET_PORT = 0x24,

		// System reset
		SYSTEM_RESET = 0x26,

		// Read data regisvter
		READ_DATA = 0x30,

		// Write data register
		WRITE_DATA = 0x31,

		// Read buffer register
		READ_FBUF = 0x32,

		// Write buffer register
		WRITE_FBUF = 0x33,

		// Get image lengths in frame buffer
		GET_FBUF_LEN = 0x34,

		// Set image lengths in frame buffer
		SET_FBUF_LEN = 0x35,

		// Control frame buffer register
		FBUF_CTRL = 0x36,

		// Motion detect on or off in comunication interface
		COMM_MOTION_CTRL = 0x37,

		// Get motion monitoring status in comunication interface
		COMM_MOTION_STATUS = 0x38,

		// Motion has been detected by comunication interface
		COMM_MOTION_DETECTED = 0x39,

		// Mirror control
		MIRROR_CTRL = 0x3A,

		// Mirror status
		MIRROR_STATUS = 0x3B,

		// Control color
		COLOR_CTRL = 0x3C,

		// Color status
		COLOR_STATUS = 0x3D,

		// Power mode control
		POWER_SAVE_CTRL = 0x3E,

		// Power save mode or not
		POWER_SAVE_STATUS = 0x3F,

		// Control AE
		AE_CTRL = 0x40,

		// AE status
		AE_STATUS = 0x41,

		// Motion control
		MOTION_CTRL = 0x42,

		// Get motion status
		MOTION_STATUS = 0x43,

		// TV output on or off control
		TV_OUT_CTRL = 0x44,

		// Add characters to OSD channels
		OSD_ADD_CHAR = 0x45,

		// Downsize Control
		DOWNSIZE_CTRL = 0x54,

		// Downsize status
		DOWNSIZE_STATUS = 0x55,

		// Get SPI flash size
		GET_FLASH_SIZE = 0x60,

		// Erase one block of the flash
		ERASE_FLASH_SECTOR = 0x61,

		// Erase the whole flash
		ERASE_FLASH_ALL = 0x62,

		// Read and show logo
		READ_LOGO = 0x70,

		// Bitmap operation
		SET_BITMAP = 0x71,

		// Write mass data at a time
		BATCH_WRITE = 0x80
	};

	enum OutputResolution {
		OR_640x480 = 0x00, OR_320x240 = 0x11, OR_160x120 = 0x22
	};

	enum BufferControl {

		// Stop current frame
		STOP_CURRENT_FRAME = 0x00,

		// Stop next frame
		STOP_NEXT_FRAME = 0x01,

		// Resume frame
		RESUME_FRAME = 0x03,

		// Step frame
		STEP_FRAME = 0x03
	};

	enum BaudRate {
		B_9600 = 0xaec8,
		B_19200 = 0x56e4,
		B_38400 = 0x2af2,
		B_57600 = 0x1c4c,
		B_115200 = 0x0da6
	};

	/**
	 * Public constructor.
	 *
	 */
	CameraVC0706(char *dev);

	/**
	 * Initializes the camera.
	 */
	int begin(int baud);

	/**
	 * Closes the camera.
	 */
	int close();

	/**
	 * Captures a frame.
	 */
	int capture();

	/**
	 * Resumes the camera.
	 */
	int resume();

	/**
	 * Gets the frame length.
	 *
	 * @return				The frame length.
	 */
	int getFrameLength();

	/**
	 * Returns a frame.
	 *
	 * @return               A frame.
	 */
	int readFrame(unsigned char *buf, int frameOffset, int bufferOffset,
			int len);

	/**
	 * En/disable horizontal mirror.
	 *
	 * @param mirror        The mirror option.
	 */
	void setHorizontalMirror(bool mirror);

	/**
	 * En/disable vertical flip.
	 *
	 * @param mirror        The vertical flip.
	 */
	void setVerticalFlip(bool flip);

	/**
	 * Sets predefined output resolution.
	 *
	 * @param resolution        The output resolution.
	 */
	bool setOutputResolution(unsigned char resolution);

	/**
	 * Get the camera version.
	 */
	float getVersion();

	/**
	 * Reset the camera
	 */
	int reset();

	/**
	 * Execute a buffer control issue.
	 *
	 * @param control               The buffer control.
	 */
	int executeBufferControl(unsigned char control);

	/**
	 * Set TV output.
	 *
	 * @param onOff               	TV output flag.
	 */
	bool setTVOutput(unsigned char onOff);

	/**
	 * Configures the boud rate.
	 *
	 * @param baudRate				The boud rate.
	 */
	bool setBoudRate(int baudRate);

private:

	void printBuff(unsigned char *buf, int c);

	int write(unsigned char *buf, int size);

	int read(unsigned char *buf, int size);

	int sendCommand(unsigned char cmd, unsigned char *args, int argc);

	bool verifyResponse(unsigned char cmd);

	int readResponse(int length);

	bool runCommand(unsigned char cmd, unsigned char *args, int argc,
			int responseLength);
};

#endif /* __RASPBERRY_DRIVER_CAMERA_VC0706_H__ */
