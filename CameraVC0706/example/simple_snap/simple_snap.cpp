#include <CameraVC0706.cpp>

void captureAndSave(CameraVC0706 *cam, char *fileName);
void showUsage();
int getBoudFromChar(char c);

int main(int argc, char *argv[]) {

	if (argc < 3) {
		showUsage();
		exit(0);
	}

	int baud = getBoudFromChar(argv[1][0]);
	printf("Unsing baud %x\n", baud);

	CameraVC0706 cam((char*) "/dev/ttyUSB0");
	if (!cam.begin(baud)) {
		printf("Cannot begin.\n");
		exit(1);
	}

	switch (argv[2][0]) {

		unsigned char c;
		case 'B':
			baud = getBoudFromChar(argv[3][0]);
			printf("Baud rate: %d\n", baud);
			cam.setBoudRate(baud);
			break;

		case 'T':
			printf("TV: ");
			if (argv[3][0] == '1') {
				printf("on\n");
				c = 1;
			} else {
				printf("off\n");
				c = 0;
			}
			cam.setTVOutput(c);
			break;

		case 'o':
			printf("Output resolution: ");
			switch(argv[3][0]) {

				case '0':
					printf("160x120\n");
					c = CameraVC0706::OR_160x120;
					break;

				case '1':
					printf("320x240\n");
					c = CameraVC0706::OR_320x240;
					break;

				case '2':
					printf("640x480\n");
					c = CameraVC0706::OR_640x480;
					break;
				default:
					printf("Error, invalid resolution.\n");
					exit(1);
			}
			cam.setOutputResolution(c);
			break;

		case 'c':
			if (argc < 4) {
				printf("File name not specified.\n");
				exit(1);
			}
			printf("Capturing to file: %s.\n", argv[3]);
			captureAndSave(&cam, argv[3]);
			break;

		default:
			printf("%f\n", cam.getVersion());
	}

	cam.close();
	return 0;
}

void captureAndSave(CameraVC0706 *cam, char *fileName) {
	cam->executeBufferControl(CameraVC0706::RESUME_FRAME);
	usleep(100000);
	if (cam->capture()) {
		FILE* destinatioFile;
		destinatioFile = fopen(fileName, "w");
		if (destinatioFile == NULL) {
			printf("@captureAndSave: cannot open file.\n");
		}
		int frameLength = cam->getFrameLength();
		printf("@captureAndSave: frame length: %d\n", frameLength);
		if (frameLength > 0) {
			unsigned char buffer[frameLength];
			int readFremeLength = cam->readFrame(&buffer[0], 0, 0,
					(int) frameLength);
			printf("@captureAndSave: readFremeLength: %d\n\n", readFremeLength);
			if (readFremeLength == frameLength) {
				fwrite(&buffer[0], sizeof(unsigned char), frameLength,
						destinatioFile);
			} else {
				printf("@captureAndSave: read %d  differs from %d.\n", readFremeLength,
						frameLength);
			}
		} else {
			printf("@captureAndSave: frame length error, returned 0.\n");
		}
	} else {
		printf("@captureAndSave: cannot campure.\n");
	}
}

void showUsage() {
	printf("program 5 B 0/1/2/3/4/5		@115200 Baud rate 9600/19200/38400/57600/115200.\n");
	printf("program 5 T 0/1       		@115200 TV off/on.\n");
	printf("program 5 o 0/1/2       	@115200 Resolution 160x120/320x240/640x480.\n");
	printf("program 5 c <filename>  	@115200 Capture.\n");
	//printf("program 5 f 0/1 0/1			@115200 Stop/Resume current/next frame.\n");
	printf("program 5 v             	@115200 Version.\n");
}

int getBoudFromChar(char c) {
	int baud = CameraVC0706::B_115200;
	switch(c) {
		case '0':
			baud = CameraVC0706::B_9600;
			break;
		case '2':
			baud = CameraVC0706::B_19200;
			break;
		case '3':
			baud = CameraVC0706::B_38400;
			break;
		case '4':
			baud = CameraVC0706::B_57600;
			break;
		case '5':
			baud = CameraVC0706::B_115200;
			break;
	}
	return baud;
}
