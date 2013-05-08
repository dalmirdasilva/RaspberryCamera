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

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>

#define VC0760_DEBUG 				1
#define VC0760_PROTOCOL_SIGN_TX     0x56
#define VC0760_PROTOCOL_SIGN_RX     0x76

#define VC0760_BUFFER_SIZE          0x100

#define VC0760_CAMERA_DELAY 		10

class CameraVC0706 {

    int fd;
    
    char dev[100];
    
    unsigned char buffer[VC0760_BUFFER_SIZE];
    
    int bufferPointer;
    
    fd_set input_fdset;
    
    unsigned char serialNumber;
    
    int framePointer;
    
public:

    enum Command {
        GEN_VERSION = 0x11, // Get Firmware version information
        SET_SERIAL_NUMBER = 0x21, // Set serial number
        SET_PORT = 0x24, // Set port
        SYSTEM_RESET = 0x26, // System reset
        READ_DATA = 0x30, // Read data regisvter
        WRITE_DATA = 0x31, // Write data register
        READ_FBUF = 0x32, // Read buffer register
        WRITE_FBUF = 0x33, // Write buffer register
        GET_FBUF_LEN = 0x34, // Get image lengths in frame buffer
        SET_FBUF_LEN = 0x35, // Set image lengths in frame buffer
        FBUF_CTRL = 0x36, // Control frame buffer register
        COMM_MOTION_CTRL = 0x37, // Motion detect on or off in comunication interface
        COMM_MOTION_STATUS = 0x38, // Get motion monitoring status in comunication interface
        COMM_MOTION_DETECTED = 0x39, // Motion has been detected by comunication interface
        MIRROR_CTRL = 0x3A, // Mirror control
        MIRROR_STATUS = 0x3B, // Mirror status
        COLOR_CTRL = 0x3C, // Control color
        COLOR_STATUS = 0x3D, // Color status
        POWER_SAVE_CTRL = 0x3E, // Power mode control
        POWER_SAVE_STATUS = 0x3F, // Power save mode or not
        AE_CTRL = 0x40, // Control AE
        AE_STATUS = 0x41, // AE status
        MOTION_CTRL = 0x42, // Motion control
        MOTION_STATUS = 0x43, // Get motion status
        TV_OUT_CTRL = 0x44, // TV output on or off control
        OSD_ADD_CHAR = 0x45, // Add characters to OSD channels
        DOWNSIZE_CTRL = 0x54, // Downsize Control
        DOWNSIZE_STATUS = 0x55, // Downsize status
        GET_FLASH_SIZE = 0x60, // Get SPI flash size
        ERASE_FLASH_SECTOR = 0x61, // Erase one block of the flash
        ERASE_FLASH_ALL = 0x62, // Erase the whole flash
        READ_LOGO = 0x70, // Read and show logo
        SET_BITMAP = 0x71, // Bitmap operation
        BATCH_WRITE = 0x80 // Write mass data at a time
    };

    enum OutputResolution {
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
     * Captures a frame.
     */
    bool capture();

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
    int readFrame(unsigned char *out, int offset, int size);

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
    void setOutputResolution(unsigned char resolution);

    /**
     * Get the camera version.
     */
    float getVersion();

    bool reset();
    
private:

    void printBuff();

    int write(unsigned char *buf, int size);

    int read(unsigned char *buf, int size);
    
    int sendCommand(unsigned char cmd, unsigned char *args, int argc);

    bool verifyResponse(unsigned char cmd);

    int readResponse(int length);

    bool runCommand(unsigned char cmd, unsigned char *args, int argc, int responseLength);
};

#endif /* __RASPBERRY_DRIVER_CAMERA_VC0706_H__ */
