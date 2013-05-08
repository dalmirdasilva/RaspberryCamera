#include "CameraVC0706.h"

CameraVC0706::CameraVC0706(char *dev) {
	int i = 0;
	while (dev[i] != '\0') {
		this->dev[i] = dev[i];
		i++;
	}
	this->dev[i] = 0;
	fd = 0;
	bufferPointer = 0;
	serialNumber = 0;
	framePointer = 0;
}

int CameraVC0706::begin(int baud) {
	struct termios options;
	fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		printf("Error - Unable to open UART.\n");
		return 0;
	}

	/**
	 * Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400,
	 * B57600, B115200, B230400, B460800, B500000, B576000, B921600,
	 * B1000000, B1152000, B1500000, B2000000, B2500000, B3000000,
	 * B3500000, B4000000
	 *
	 * CSIZE:- CS5, CS6, CS7, CS8
	 * CLOCAL - Ignore modem status lines
	 * CREAD - Enable receiver
	 * IGNPAR = Ignore characters with parity errors
	 * ICRNL - Map CR to NL on input
	 * PARENB - Parity enable
	 * PARODD - Odd parity (else even)
	 */
	tcgetattr(fd, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR | ICRNL;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);
	//reset();
	return 1;
}

bool CameraVC0706::capture() {
	unsigned char args[] = { 0x01, 0x00 };
	return runCommand(FBUF_CTRL, args, sizeof(args), 5);
}

int CameraVC0706::readFrame(unsigned char *out, int offset, int size) {
	unsigned char args[] = { 0x0c, 0x00, 0x0a, ((offset >> 24) & 0xff), ((offset
			>> 16) & 0xff), ((offset >> 8) & 0xff), size & 0xff, ((size >> 24)
			& 0xff), ((size >> 16) & 0xff), ((size >> 8) & 0xff), size & 0xff,
			(VC0760_CAMERA_DELAY >> 8) & 0xff, VC0760_CAMERA_DELAY & 0xff };

	if (!runCommand(READ_FBUF, args, sizeof(args), 5)) {
		return 0;
	}
	int pointer = 0;
	while (pointer < size) {
		pointer += read(&out[pointer], size);
	}
	return pointer;
}

int CameraVC0706::getFrameLength() {
	int length;
	unsigned char args[] = { 0x01, 0x00 };
	if (!runCommand(GET_FBUF_LEN, args, sizeof(args), 9)) {
		return 0;
	}
	length = buffer[5];
	length <<= 8;
	length |= buffer[6];
	length <<= 8;
	length |= buffer[7];
	length <<= 8;
	length |= buffer[8];
	return length;
}

void CameraVC0706::setHorizontalMirror(bool mirror) {
}

void CameraVC0706::setVerticalFlip(bool flip) {
}

void CameraVC0706::setOutputResolution(unsigned char resolution) {
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

		}
	} else {

#if VC0760_DEBUG == 1
		printf("UART file descriptor is closed.\n");
#endif

	}
	return txLength;
}

int CameraVC0706::read(unsigned char *buf, int size) {
	int rxLength = ::read(fd, buffer, size);
	if (rxLength < 0) {

#if VC0760_DEBUG == 1
		printf("Error on read.\n");
#endif

	} else if (rxLength == 0) {

#if VC0760_DEBUG == 1
		printf("No data received on read.\n");
#endif

	} else {
		buffer[rxLength] = '\0';

#if VC0760_DEBUG == 1
		printf("%i bytes read.\n", rxLength);
#endif

	}
	return rxLength;
}

bool CameraVC0706::runCommand(unsigned char cmd, unsigned char *args, int argc,
		int responseLength) {
	int length;
	length = sendCommand(cmd, args, argc);

#if VC0760_DEBUG == 1
	printf("sendCommand returned %d.\n", length);
#endif

	if (!length) {
		return false;
	}
	usleep(10000);
	length = readResponse(responseLength);

#if VC0760_DEBUG == 1
	printf("readResponse returned %d.\n", length);
#endif

	if (!length) {
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
	if ((buffer[0] != VC0760_PROTOCOL_SIGN_RX) || (buffer[1] != serialNumber)
			|| (buffer[2] != cmd) || (buffer[3] != 0x00)) {
		return false;
	}
	return true;
}

int CameraVC0706::readResponse(int length) {
	bufferPointer = read(buffer, length);
	printBuff();
	return bufferPointer;
}

void CameraVC0706::printBuff() {
#if VC0760_DEBUG == 1
	printf("Printing buffer:\n");
	for (int i = 0; i < bufferPointer; i++) {
		printf("\t%d: 0x%x\n", i, buffer[i]);
	}
#endif
}

bool CameraVC0706::reset() {
	unsigned char args[] = {0x0};
	return runCommand(SYSTEM_RESET, args, 1, 5);
}

float CameraVC0706::getVersion() {
	int i = 0;
	unsigned char args[] = { 0x00 };
	if (!runCommand(GEN_VERSION, args, sizeof(args), VC0760_BUFFER_SIZE)) {
		return 0.0;
	}
	float version = 0.0;
	while (buffer[i++] != ' ')
		;
	version += buffer[i] - '0';
	version += 0.1 * (buffer[i + 2] - '0');
	version += 0.01 * (buffer[i + 3] - '0');
	return version;
}
