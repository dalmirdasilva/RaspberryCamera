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
#include <time.h>

#define VC0760_DEBUG 				1
#define VC0760_PROTOCOL_SIGN_TX     0x56
#define VC0760_PROTOCOL_SIGN_RX     0x76

#define VC0760_RX_BUFFER_SIZE       0xff
#define VC0760_CAMERA_DELAY 		0x0100

class CameraVC0706 {

	int fd;

	char dev[100];

	unsigned char rxBuffer[VC0760_RX_BUFFER_SIZE];

	int rxBufferPointer;

	unsigned char serialNumber;

	int framePointer;
	
	int baudRate;

public:
	
	enum ControlBy {
		GPIO = 0x00,
		UART = 0x01,
	};
	
	enum MotionControl {
		
		// Motion control and enabling control
		MOTION_CONTROL = 0, 
		
		// Alarm-output attribute
		ALARM_ATTRIBUTE = 1,
		
		// Alarm-output enabling control
		ALARM_ENABLING = 2,
		
		// Alarm-output control
		ALARM_CONTROL = 3
	};

	
	enum ColorControlMode {
		
		// Automatically step black-white and color.
		AUTO_STEP_BLACK_WHITE = 0,
		
		// Manually step color, select color.
		MANUAL_STEP_SELECT_COLOR = 1,
		
		// Manually step color, select black-white.
		MANUAL_STEP_SELECT_BLACK_WHITE = 2
	};

	enum Command {

		// Get Firmware version information
		GEN_VERSION = 0x11,

		// Set serial number
		SET_SERIAL_NUMBER = 0x21,

		// Set port
		SET_PORT = 0x24,

		// System reset
		SYSTEM_RESET = 0x26,

		// Read data register
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

		// Motion detect on or off in communication interface
		COMM_MOTION_CTRL = 0x37,

		// Get motion monitoring status in communication interface
		COMM_MOTION_STATUS = 0x38,

		// Motion has been detected by communication interface
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

		// Add characters to OSD channels (unsupported by the VC0706 firmware)
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
		RES_640X480 = 0x00, RES_320X240 = 0x11, RES_160X120 = 0x22
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
	bool begin(int baud);

	/**
	 * Closes the camera.
	 */
	bool close();

	/**
	 * Captures a frame.
	 */
	bool capture();

	/**
	 * Resumes the camera.
	 */
	bool resume();

	/**
	 * Command function : control downsize attribute
	 *
	 * Command format :0x56+serial number+0x53+0x01+control item(1 byte)control item:zooming image proportion
	 *
	 * <pre>
	 * Bit[1:0]:width zooming proportion
	 *      2b'00:1:1, no zoom
	 *      2b'01:1:2, the proportion is 1/2.
	 *      2b'10:1:4, the proportion is 1/4.
	 *      2b'11:reservation
	 *
	 * Bit[3:2]:height zooming proportion
	 *      2b'00:1:1, no zoom
	 *      2b'01:1:2, the proportion is 1/2.
	 *      2b'10:1:4, the proportion is 1/4.
	 *      2b'11:reservation
	 * </pre>
	 *
	 * Notice:
	 *
	 * 1. The image width must be the multiple of 16 in FBUF, image height is the multiple of 8, so the
	 * configuration information could satisfy the condition.
	 * 2. The zooming proportion of image height is not more than the zooming proportion of width.
	 * Return format : 0x76+serial number+0x53+0x00+0x00
	 *
	 * @param widthDownSize                 The width downsize.
	 * @param heightDownSize                The height downsize.
	 */
    bool setDownSize(unsigned char widthDownSize, unsigned char heightDownSize);

	/**
	 * Command function : get downsize status
	 *
	 * Command format : 0x56+serial number+0x54+0x00 control item:zooming image proportion
	 *
	 * <pre>
	 * Bit[1:0]:width zooming proportion
	 *      2b'00:1:1, no zoom
	 *      2b'01:1:2, the proportion is 1/2.
	 *      2b'10:1:4, the proportion is 1/4.
	 *      2b'11:reservation
	 *
     * Bit[3:2]:height zooming proportion
     *      2b'00:1:1, no zoom
     *      2b'01:1:2, the proportion is 1/2.
     *      2b'10:1:4, the proportion is 1/4.
     *      2b'11:reservation
     * </pre>
     *
     * Return format : 0x76+serial number+0x54+0x00+0x01+control item(1 byte)
	 */
	unsigned char getDownSize();

	/**
	 * Gets the frame length.
	 * 
	 * Command function :get byte-lengths inFBUF
	 * Command format :0x56+serial number+0x34+0x01+FBUF type(1 byte)
	 * 
	 * <pre>
	 * FBUF type:current frame or next frame
	 * 		0:current frame
	 * 		1:next frame
	 * </pre>
	 * 
	 * Return format :
	 * OK:0x76+serial number+0x34+0x00+0x04+FBUF data-lengths(4 bytes)
	 *
	 * @return				The frame length.
	 */
	int getFrameLength();

	/**
	 * Returns a frame.
	 * 
	 * Command function :read image data from FBUF.
	 * Command format :0x56+serial number+0x32+0x0C+FBUF type(1 byte)+control mode(1 byte)
	 *  +starting address(4 bytes)+data-length(4 bytes)+delay(2 bytes)
	 * 
	 * <pre>
	 * FBUF type:current frame or next frame
	 * 		0:current frame
	 * 		1:next frame
	 * 
	 * Control mode:the mode by which image data transfer
	 * 		Bit0:0:data transfer by MCU mode
	 * 			 1:data transfer by DMA mode
	 * 		Bit[2:1]:2'b11
	 * 		Bit3: 1'b11
	 * </pre>
	 * 
	 * Starting address: the address in fbuf to store the image data.
	 * Data-length:the byte number ready to read, it must be the multiple of 4.
	 * Delay:the delay time between command and data, the unit is 0.01 millisecond.
	 * Return format :
	 * Ok:if execute right, return 0x76+serial number+0x32+0x00+0x00, the following is image data,
	 * at last, return 0x76+serial number+0x32+0x00+0x00 again.
	 *
	 * @return               A frame.
	 */
	int readFrame(unsigned char *buf, int frameOffset, int bufferOffset,
			int len);

	/**
	 * En/disable horizontal mirror.
	 * 
	 * Command function : control show status of sensor mirror.
	 * Command format :0x56+serial number+0x3A+0x02+control mode(1 byte)+Mirror mode(1 byte)
	 * 
	 * <pre>
	 * Control mode:
	 * 		0:control mirror by GPIO.
	 * 		1:control mirror by UART.
	 * </pre>
	 * 
	 * Mirror mode:whether show mirror by UART, it is effective only with UART.It needs GPIO 
	 * value to set with GPIO control.
	 * 
	 * <pre>
	 * 		0:do not show mirror
	 * 		1:show mirror
	 *</pre>
	 * 
	 * @param by						The mirror control.
	 * @param mirrorMode				The mirror mode.
	 */
	bool setHorizontalMirror(unsigned char by, unsigned char mirrorMode);
	
	/**
	 * Command function : get show status of sensor mirror.
	 * 
	 * Command format :0x56+serial number+0x3B+0x00
	 * 
	 * Control mode:
	 * 		0:control mirror by GPIO.
	 * 		1:control mirror by UART.
	 * 
	 * Mirror mode:whether show mirror by UART, it is effective only with UART.
	 * It needs GPIO value to set with GPIO control.
	 * 		0:do not show mirror
	 * 		1:show mirror
	 * 
	 * Return format :
	 * 0x76+serial number+0x3B+0x00+0x02+control mode(1 byte)+Mirror mode(1 byte)
	 * 
	 * @return status						bit0 is the control mode, and the 
	 * 										bit1 is the mirror mode. 
	 */
	unsigned char getHorizontalMirrorStatus();

	/**
	 * Command function : color control mode and show mode
	 * Command format :0x56+serial number+0x3C+0x02+control mode(1 byte)+show mode(1 byte)
	 * 
	 * <pre>
	 * Control mode:
	 * 		0:control color by GPIO.
	 * 		1:control color by UART.
	 * </pre>
	 * Show mode:show different color by UART, it is effective only with UART.
	 * 
	 * <pre>
	 * It needs Mirror value to set with GPIO control.
	 * 		0:automatically step black-white and color.
	 * 		1:manually step color, select color.
	 * 		2:manually step color, select black-white.
	 * </pre>
	 * 
	 * Return format :
	 * OK:0x76+serial number+0x3C+0x00+0x00
	 * 
	 * @param by							The color control (UART or GPIO).
	 * @param colorControlMode				The color control mode.
	 */
	bool setColorControl(unsigned char by, unsigned char colorControlMode);
  
    /**
     * Command function : get color control mode and show mode
     * 
     * Command format :0x56+serial number+0x3D+0x00
     * 
     * <pre>
     * Control mode:
     *      0:control color by GPIO.
     *      1:control color by UART.
     * 
     * Show mode:show current color by UART.
     *      0:automatically step black-white and color.
     *      1:manual step color, select color.
     *      2:manual step color, select black-white.
     * </pre>
     * 
     * Return format :
     * 
     * 0x76+serial number+0x3D+0x00+0x03+control mode(1 byte)+show mode(1 byte)+current color(1 byte)
     * 
     * @return status                   bit0 Control mode (GPIO = 0, UART = 1)
     *                                  bit[1,2] Show mode
     *                                      00:automatically step black-white and color.
     *                                      01:manual step color, select color.
     *                                      10:manual step color, select black-white.
     */ 
	unsigned char getColorControlStatus();

	/**
	 * Sets predefined output resolution.
	 *
	 * @param resolution        The output resolution.
	 */
	bool setOutputResolution(unsigned char resolution);
	
	/**
	 * Sets the motion detection.
	 * 
	 * Command function :motion detect on or off in communication interface
	 * Command format :0x56+serial number+0x37+0x01+control flag(1 byte)
	 * 
	 * <pre>
	 * control flag:
	 * 		0:stop motion monitoring
	 * 		1:start motion monitoring
	 * </pre>
	 * Error:0x76+serial number+0x37+0x03+0x00
	 * 
	 * @param monitor			The flag.
	 */
	bool setMotionMonitoring(bool monitor);
	
	/**
	 * Gets the motion status.
	 * 
	 * Command function :get motion monitoring status in communication interface.
	 * Command format :0x56+serial number+0x38+0x00
	 * 
	 * Return format :
	 * 
	 * OK:0x76+serial number+0x38+0x00+0x01+control flag(1 byte)
	 * 
	 * <pre>
	 * control flag:
	 * 		0:stop motion monitoring
	 * 		1:start motion monitoring
	 * </pre>
	 * Error:0x76+serial number+0x37+0x03+0x00
	 * 
	 * @param return			The flag.
	 */
	bool getMotionMonitoringStatus();
	
	/**
	 * Command function : motion control 
	 * 
	 * Command format :0x56+serial number+0x42+data-lengths+motion attribute+control item 
	 * 
	 * <pre>
	 * motion attribute:
	 * 
	 * 		0:motion control and enabling control
	 * 		1:alarm-output attribute
	 * 		2:alarm-output enabling control
	 * 		3:alarm-output control 
	 * 
	 * control item:
	 * 		> motion control and enabling control
	 * 		The first byte:
	 * 			0:GPIO
	 * 			1:UART
	 * 		The second byte:
	 * 			0:forbid motion monitoring
	 * 			1:start motion monitoring
	 *		> alarm-output attribute
	 * 		The first byte:
	 * 			bit0:alarm type
	 * 			0:stop alarming at a certain time.
	 * 			1:alarm at all times.
	 * 			Bit1:alarm electrical level
	 * 			0:it is low level until alarm.
	 * 			1:it is high level until alarm.
	 * 		The second and third byte mean the alarm time, the lower byte follows the higher byte, the unit is 10 millisecond.
	 * 		> alarm-output enabling control
	 * 			The first byte:
	 * 			0:forbid alarm-output
	 * 			1:enable alarm-output
	 * 		> alarm-output control
	 * 			The first byte:
	 * 			0:stop alarm-output
	 * 			1:start alarm-output
	 * </pre>
	 * 
	 * Return format :
	 * OK: 0x76+serial number+0x42+0x00+0x00
	 * Error:0x76+serial number+0x42+0x03+0x00
	 * E.g.
	 * > 0x56+0x00+0x42+0x03+0x00+0x01+0x01
	 * Enable motion monitoring by MCU UART, and open it.
	 * > 0x56+0x00+0x42+0x03+0x00+0x01+0x00
	 * Enable motion monitoring by MCU UART, and stop it.
	 * > 0x56+0x00+0x42+0x03+0x00+0x00+0x00
	 * Enable motion monitoring by GPIO.
	 * > 0x56+0x00+0x42+0x04+0x01+0x02+0x00+0x64
	 * Set alarm-output attribute.
	 * > 0x56+0x00+0x42+0x02+0x02+0x01
	 * Enable alarm-output control.
	 * > 0x56+0x00+0x42+0x02+0x02+0x00
	 * Disallow alarm-output control.
	 * > 0x56+0x00+0x42+0x02+0x03+0x01
	 * Start alarm-output.
	 * > 0x56+0x00+0x42+0x02+0x03+0x00
	 * Stop alarm-output.
	 */
	bool setMotionControl(unsigned char motionControl, unsigned char param0, unsigned char param1);
	
	/**
	 * Polling for motion detection.
	 * 
	 * Command function : detect motion
	 * 
	 * Command format :
	 * After starting motion monitoring, once system detects motion, it will send the command.
	 * Return format :0x76+serial number+0x39+0x00
	 * 
	 * E.g.
	 * 0x76+0x00+0x39+0x00 detect motion
	 * 
	 * It is an active command that system send to control terminal.
	 * 
	 * @param timeout			The timeout to wait.
	 * @param callback			Function pointer.
	 */
	bool pollMotionMonitoring(int timeout, void (*callback)(void *));
	
	/**
	 * Command function : add OSD characters to channels(channel 1)
	 * 
	 * Command format :0x56+serial number+0x45+data-length+character 
	 * 	number(1 byte)+starting address(1 byte)+characters(n characters)character number:
	 * 	the number of characters which continuously are written to channels, the most is 14.
	 * 
	 * <pre>
	 * starting address:the starting place from which characters show. The format is as follows.
	 * 		Bit[4-0]:Y-coordinate
	 * 		Bit[6-5]:X-coordinate
	 * </pre>
	 * 
	 * Characters:the characters ready to show. It is VC0706 OSD characters.
	 * Return format :
	 * OK: 0x76+serial number+0x45+0x00+0x00
	 * 
	 * @param x						The x position.
	 * @param y						The y position.
	 * @param str					The string to be used.
	 * @param len					How many char to use.
	 */
	bool setOsdCharacters(unsigned char x, unsigned char y, unsigned char *str, unsigned char len);
	
	/**
	 * Set image compression.
	 * 
	 * @param compression			The compression.
	 */
	bool setCompression(unsigned char compression);

	/**
	 * Get image compression.
	 * 
	 * @return						The compression.
	 */
	unsigned char getCompression();

	/**
	 * Get the camera version.
	 * 
	 * Command function :Get Firmware version information
	 * Command format :0x56+Serial number+0x11+0x00
	 * Return format :0x76+Serial number+0x11+0x00+0x0B+"VC0706 1.00"
	 * 
	 * @return 					The float version.
	 */
	float getVersion();

	/**
	 * Reset the camera
	 */
	bool reset();

	/**
	 * Execute a buffer control issue.
	 * 
	 * Command function :control frame buffer register
	 * Command format :0x56+serial number+0x36+0x01+control flag(1 byte) control flag:
	 * <pre>
	 * 		0:stop current frame
	 * 		1:stop next frame
	 * 		2:resume frame
	 * 		3:step frame
	 * </pre>
	 * Return format :
	 * OK:0x76+serial number+0x36+0x00+0x00
	 *
	 * @param control               The buffer control.
	 */
	bool executeBufferControl(unsigned char control);

	/**
	 * Set TV output.
	 *
	 * @param onOff               	TV output flag.
	 */
	bool setTVOutput(unsigned char onOff);

	/**
	 * Configures the baud rate.
	 *
	 * Command function :Set the property of communication interface
	 * Command format :0x56+Serial number+0x24+Data-length+interface type1byte)+configuration data
	 * 
	 * Such as set MCU UART:
	 * 0x56+Serial number+0x24+0x03+0x01+S1RELH(1byte)+S1RELL(1byte) interface type:
	 * <pre>
	 * 		0x01:MCU UART
	 * </pre>
	 * 
	 * Return format :
	 * OK: 0x76+Serial number+0x24+0x00+0x00
	 * 
	 * @param baudRate				The baud rate.
	 */
	bool setBoudRate(int baudRate);

	/**
	 * Runs a command.
	 * 
	 * @param cmd					The command to be runned.
	 * @param args					Buffer of the command params.
	 * @param argc 					How many bytes the buffer has (the command args size).
	 * @param responseLength		The expected response length.
	 */
	bool executeCommand(unsigned char cmd, unsigned char *args, int argc,
			int responseLength);

private:

	/**
	 * Utility function.
	 * 
	 * @param buf				The buffer to be debuged.
	 * @param c					How many bytes will be printed.
	 */
	void printBuff(unsigned char *buf, int c);

	/**
	 * Writes to UART.
	 * 
	 * @param buf				Buffer from data will come from.
	 * @param size				How many bytes will tried to write.
	 */
	int write(unsigned char *buf, int size);

	/**
	 * Reads from UART.
	 * 
	 * @param buf				Buffer where data will be read to.
	 * @param size				How many bytes will tried to read.
	 */
	int read(unsigned char *buf, int size);

	/**
	 * Receive command format :
	 * 
	 * Protocol sign(1byte)+Serial number(1byte)+Command(1byte)+Data-lengths(1byte)+Data(0~16bytes)
	 * 
	 * @param cmd				The command.
	 * @param args				The command data array.
	 * @param argc				The command data length.
	 */
	int sendCommand(unsigned char cmd, unsigned char *args, int argc);

	/**
	 * Protocol sign(1byte)+Serial number(1byte)+Command(1byte)+Status(1byte)+Data-lengths(1byte)+Data(0~16bytes)
	 * 
	 * @param cmd				The command to check the response.
	 * @return 					True if there is a correct response, false otherwise.
	 */
	bool verifyResponse(unsigned char cmd);

	/**
	 * Reads data and put into the rxBuffer.
	 * 
	 * Adjust the current rxBuffer pointer.
	 * 
	 * @param length			How many data will be read.
	 * @return 					How many data was actually read.
	 */
	int readResponse(int length);
};

#endif /* __RASPBERRY_DRIVER_CAMERA_VC0706_H__ */
