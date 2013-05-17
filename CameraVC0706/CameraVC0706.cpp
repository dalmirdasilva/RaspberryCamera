#include "CameraVC0706.h"

CameraVC0706::CameraVC0706(char *dev) {
	int i = 0;
	while (dev[i] != '\0') {
		this->dev[i] = dev[i];
		i++;
	}
	this->dev[i] = 0;
	fd = 0;
	rxBufferPointer = 0;
	serialNumber = 0x00;
	framePointer = 0;
	frameTransferControl = MCU; 
}

int CameraVC0706::begin(int baud) {
	struct termios options;
	fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		printf("Error - Unable to open UART.\n");
		return 0;
	}
	int baudRate = B115200;
	switch(baud) {
		case B_9600:
			baudRate = B9600;
			break;
		case B_19200:
			baudRate = B19200;
			break;
		case B_38400:
			baudRate = B38400;
			break;
		case B_57600:
			baudRate = B57600;
			break;
		default:
			baudRate = B115200;
			break;
	}
    cfsetispeed(&options, baudRate);
    cfsetospeed(&options, baudRate);
	tcgetattr(fd, &options);
	options.c_cflag = baudRate | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);
	return 1;
}

int CameraVC0706::close() {
	return ::close(fd);
}

int CameraVC0706::capture() {
	return executeBufferControl(STOP_CURRENT_FRAME);
}

int CameraVC0706::resume() {
	return executeBufferControl(RESUME_FRAME);
}

int CameraVC0706::executeBufferControl(unsigned char control) {
	unsigned char args[] = {0x01, (unsigned char)(control & 0x03)};
	return runCommand(FBUF_CTRL, args, sizeof(args), 5);
}

int CameraVC0706::readFrame(unsigned char *buf, int frameOffset, int bufferOffset, int len) {
	int bytesRead = 0;
	unsigned char args[] = {0x0c, 0x00, (0x0f & frameTransferControl),
        ((frameOffset >> 24) & 0xff), ((frameOffset >> 16) & 0xff), ((frameOffset >> 8) & 0xff), frameOffset & 0xff,
        ((len >> 24) & 0xff), ((len >> 16) & 0xff), ((len >> 8) & 0xff), len & 0xff,
        (VC0760_CAMERA_DELAY >> 8) & 0xff, VC0760_CAMERA_DELAY & 0xff};

	if (!runCommand(READ_FBUF, args, sizeof(args), 5)) {
		return 0;
	}
	while (bytesRead < len) {
		usleep(100000);
		bytesRead += read(&buf[bufferOffset + bytesRead], len - bytesRead);
	}
	readResponse(5);
	return bytesRead;
}

int CameraVC0706::getFrameLength() {
	int frameLength;
	unsigned char args[] = {0x01, 0x00};
	if (!runCommand(GET_FBUF_LEN, args, sizeof(args), 9) && rxBuffer[4] == 0x04) {
		return 0;
	}
	frameLength = rxBuffer[5];
	frameLength <<= 8;
	frameLength |= rxBuffer[6];
	frameLength <<= 8;
	frameLength |= rxBuffer[7];
	frameLength <<= 8;
	frameLength |= rxBuffer[8];
	return frameLength;
}

bool CameraVC0706::setHorizontalMirror(unsigned char by, unsigned char mirrorMode) {
	unsigned char args[] = {0x02, (by & 0x01), (mirrorMode & 0x01)};
	return runCommand(MIRROR_CTRL, args, sizeof(args), 5);
}

unsigned char CameraVC0706::getHorizontalMirrorStatus() {
	unsigned char args[] = {0x00};
	bool run = runCommand(MIRROR_STATUS, args, sizeof(args), 7);
	unsigned char status = 0;
	if (run) {
		status = rxBuffer[6] & 0x01 | (rxBuffer[5] << 1) & 0x02;
	}
	return status;
}

bool CameraVC0706::setColorControl(unsigned char by, unsigned char colorControlMode) {
	unsigned char args[] = {0x02, (by & 0x01), (colorControlMode & 0x01)};
	return runCommand(COLOR_CTRL, args, sizeof(args), 5);
}

unsigned char CameraVC0706::getColorControlStatus() {
	unsigned char args[] = {0x00};
	bool run = runCommand(COLOR_STATUS, args, sizeof(args), 8);
    unsigned char status = 0;
    if (run) {
        status = rxBuffer[5] & 0x01 | (rxBuffer[6] << 2) & 0x06;
    }
    return status;
}

bool CameraVC0706::setOutputResolution(unsigned char resolution) {
	unsigned char args[] = {0x05, 0x04, 0x01, 0x00, 0x19, (unsigned char) (resolution & 0x03)};
	return runCommand(WRITE_DATA, args, sizeof(args), 5);
}

bool CameraVC0706::setMotionMonitoring(bool monitor) {
	unsigned char args[] = {0x01, (unsigned char) (monitor & 0x01)};
	return runCommand(COMM_MOTION_CTRL, args, sizeof(args), 5);
}

bool CameraVC0706::getMotionMonitoringStatus() {
	unsigned char args[] = {0x00};
	bool (runCommand(COMM_MOTION_STATUS, args, sizeof(args), 6)) && rxBuffer[5];
}

bool CameraVC0706::pollMotionMonitoring(int timeout, void (*callback)(void *)) {
	time_t start, now;
	bool detected = 0;
	time(&start);
	do {
		readResponse(4);
		detected = verifyResponse(COMM_MOTION_DETECTED);
		if (detected && callback != 0) {
			callback(this);
		} else {
			time(&now);
		}
	} while(((now - start) < timeout) && !detected);
}

int CameraVC0706::write(unsigned char *buf, int size) {
	int txLength = 0;
	if (fd != -1) {

#if VC0760_DEBUG == 1
		printf("About to write %d bytes.\n", size);
#endif

		txLength = ::write(fd, &buf[0], size);
		if (txLength < 0) {

#if VC0760_DEBUG == 1
			printf("UART TX error\n");
#endif

		} else if(txLength != size) {

#if VC0760_DEBUG == 1
            printf("Sent bytes (%d) differs from the size to be send (%d).\n", txLength, size);
#endif

       }
	} else {

#if VC0760_DEBUG == 1
		printf("UART file descriptor is closed.\n");
#endif

        return -1;
	}
	return txLength;
}

int CameraVC0706::read(unsigned char *buf, int size) {
	int rxLength = ::read(fd, buf, size);
	
#if VC0760_DEBUG == 1
	if (rxLength < 0) {
		printf("Error on read.\n");
	} else if (rxLength == 0) {
		printf("No data received on read.\n");
	} else if (rxLength != size) {
		printf("Read bytes: %d differs from the size to be read: %d.\n", rxLength, size);
   } else {
		printf("It matches! %i bytes read when expecting %i.\n", rxLength, size);
	}
#endif

	return rxLength;
}

bool CameraVC0706::runCommand(unsigned char cmd, unsigned char *args, int argc,
		int responseLength) {
	if (!sendCommand(cmd, args, argc)) {
		return false;
	}
	usleep(50000);
	if (!readResponse(responseLength)) {
		return false;
	}
	if (!verifyResponse(cmd)) {
		return false;
	}
	return true;
}

int CameraVC0706::sendCommand(unsigned char cmd, unsigned char *args,
		int argc) {
	int length;
	unsigned char buf[3 + argc];
	buf[0] = VC0760_PROTOCOL_SIGN_TX;
	buf[1] = serialNumber;
	buf[2] = cmd;
	memcpy(&buf[3], args, argc);
	printf("send command");
	printBuff(buf, sizeof(buf));
	length = write(buf, sizeof(buf));

#if VC0760_DEBUG == 1
	printf("%d bytes written.\n", length);
#endif

	if (length != (int) sizeof(buf)) {

#if VC0760_DEBUG == 1
		printf("Cannot write. Returned %d\n", length);
#endif

		return 0;
	}
	return length;
}

bool CameraVC0706::verifyResponse(unsigned char cmd) {
	if ((rxBuffer[0] != VC0760_PROTOCOL_SIGN_RX) || (rxBuffer[1] != serialNumber)
			|| (rxBuffer[2] != cmd) || (rxBuffer[3] != 0x00)) {
		return false;
	}
	return true;
}

int CameraVC0706::readResponse(int length) {
	rxBufferPointer = read(rxBuffer, length);
	printBuff(rxBuffer, rxBufferPointer);
	return rxBufferPointer;
}

void CameraVC0706::printBuff(unsigned char *buf, int c) {
#if VC0760_DEBUG == 1
	printf("Printing buffer:\n");
	for (int i = 0; i < c; i++) {
		printf("\t%d: 0x%x\n", i, buf[i]);
	}
#endif
}

bool CameraVC0706::reset() {
	unsigned char args[] = {0x00};
	return runCommand(SYSTEM_RESET, args, sizeof(args), 5);
}

float CameraVC0706::getVersion() {
	int i = 0;
	unsigned char args[] = {0x00};
	if (!runCommand(GEN_VERSION, args, sizeof(args), VC0760_RX_BUFFER_SIZE)) {
		return 0.0;
	}
	float version = 0.0;
	while (rxBuffer[i++] != ' ')
		;
	version += rxBuffer[i] - '0';
	version += 0.1 * (rxBuffer[i + 2] - '0');
	version += 0.01 * (rxBuffer[i + 3] - '0');
	return version;
}

bool CameraVC0706::setTVOutput(unsigned char onOff) {
	unsigned char args[] = {0x01, onOff};
	return runCommand(TV_OUT_CTRL, args, sizeof(args), 5);
}

bool CameraVC0706::setBoudRate(int baudRate) {
	BaudRate boud = (BaudRate) baudRate;
	unsigned char args[] = {0x03, 0x01, (boud >> 8) & 0xff, boud & 0xff};
	return runCommand(SET_PORT, args, sizeof(args), 5);
}
