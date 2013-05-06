
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

    struct termios old_flags; 
	struct termios term_attr;
	printf("begin dev: %s\n", dev);
    if ((fd = open(dev, O_RDWR | O_NONBLOCK)) == -1) {
        perror("Can't open device ");
        return 1;
    } 
	fcntl(fd, F_SETFL, O_NONBLOCK);

    // Configurare RS232
    if (tcgetattr(fd, &term_attr) != 0) {
        perror("tcgetattr() failed"); 
        return 1;
    } 
    
    // Save old flags
    old_flags = term_attr; 
    cfsetispeed(&term_attr, baud); 
    cfsetospeed(&term_attr, baud); 
    cfmakeraw(&term_attr);

	term_attr.c_iflag = 0; 
	term_attr.c_oflag = 0; 
	term_attr.c_lflag = 0;
	term_attr.c_cflag = 0;
 
    // Finished after one bye 
    term_attr.c_cc[VMIN] = 1;
    
    // or 800ms time out 
    term_attr.c_cc[VTIME] = 8;

    // Added
    term_attr.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    
    // Using flow control via CTS/RTS 
    term_attr.c_cflag |= (baud | CS8 | CRTSCTS | CLOCAL | HUPCL);

	term_attr.c_oflag |= (OPOST | ONLCR); 

	// Save old configuration
    old_flags = term_attr; 
    term_attr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
                                                            
    if (tcsetattr(fd, TCSAFLUSH, &term_attr) != 0) { 
        perror("tcsetattr() failed"); 
        return 1; 
    } 

    // Change standard input
    if (tcgetattr(STDIN_FILENO, &term_attr) != 0) {
        perror("tcgetattr() failed"); 
        return 1; 
    } 

    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &term_attr) != 0) {
        perror("terminal: tcsetattr() failed"); 
    }
    
    // Select the first channel 1
    FD_SET(fd, &input_fdset);

    printf("fd: %d\n", fd);

    return 0;
}

bool CameraVC0706::capture() {
	return false;
}

int CameraVC0706::readFrame(unsigned char *out) {
	return 0;
}

void CameraVC0706::setHorizontalMirror(bool mirror) {
}

void CameraVC0706::setVerticalFlip(bool flip) {
}

void CameraVC0706::setOutputResolution(unsigned char resolution) {
}

int CameraVC0706::write(unsigned char *buf, int size) {

    int check;

    check = ::write(fd, buf, size);

    // Waits until all output written to the object referred to 
    // by fildes is transmitted
    tcdrain(fd);

    if (check < 0) {
        perror("write failed"); 
        close(fd);
        return -1;
    } else if(check == 0) {
        perror("no bytes transmitted");
        close(fd);
        return 0;
    }
    return check;                                                                       	                                
}

int CameraVC0706::read(unsigned char *buf, int size) {
	int state = 1;
	int received = 0;
	for (int o = 0; o < 10; o++) {
		printf("_: %d\n", ::read(fd, &buf[received], 10));
	}
    while(state > 0 && received < size) {
        state = ::read(fd, &buf[received], 1);
        printf("state: %d\n", state);
        if(state > 0) {
            received++; 
        }
    } 
    return received;
}

bool CameraVC0706::runCommand(unsigned char cmd, unsigned char *args, int argc, int responseLength) {
    sendCommand(cmd, args, argc);
    if (!readResponse(responseLength)) {
        return false;
    }
    if (!verifyResponse(cmd)) {
        return false;
    }
    return true;
}

void CameraVC0706::sendCommand(unsigned char cmd, unsigned char args[], int argc) {
    unsigned char b[] = {VC0760_PROTOCOL_SIGN_TX, serialNumber, cmd};
    printf("write(b, 3): %d\n", write(b, 3));
    printf("write(args, argc): %d\n", write(args, argc));
}


bool CameraVC0706::verifyResponse(unsigned char cmd) {
    if ((buffer[0] != VC0760_PROTOCOL_SIGN_RX) ||
        (buffer[1] != serialNumber) ||
        (buffer[2] != cmd) ||
        (buffer[3] != 0x00)) {
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
	printf("Printing buffer:\n");
	for (int i = 0; i < bufferPointer; i++) {
		printf("0x%x\n", buffer[i]);
	}
}

float CameraVC0706::getVersion() {
    int i = 0;
    unsigned char args[] = {0x01};
    sendCommand(GEN_VERSION, args, sizeof(args));
    if (!readResponse(VC0760_BUFFER_SIZE)) {
        return 0.0;
    }
    if (!verifyResponse(GEN_VERSION)) {
        return 0.0;
    }
    float version = 0.0;
    while (buffer[i++] != ' ');
    version += abs('0' - buffer[i - 1]);
    version += 0.1 * abs('0' - buffer[i + 1]);
    version += 0.01 * abs('0' - buffer[i + 2]);
    return version;
}
