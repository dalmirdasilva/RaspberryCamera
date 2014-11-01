################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CameraVC0706/example/simple_snap/simple_snap.cpp 

OBJS += \
./CameraVC0706/example/simple_snap/simple_snap.o 

CPP_DEPS += \
./CameraVC0706/example/simple_snap/simple_snap.d 


# Each subdirectory must supply rules for building sources it contributes
CameraVC0706/example/simple_snap/simple_snap.o: ../CameraVC0706/example/simple_snap/simple_snap.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I/opt/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/include -I/opt/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/libc/usr/include -I/opt/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/lib/gcc/arm-linux-gnueabihf/4.7.2/include-fixed -I/opt/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/lib/gcc/arm-linux-gnueabihf/4.7.2/include -I/opt/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/lib/gcc/arm-linux-gnueabihf/4.7.2/finclude -I"/storage/raspberry/driver/camera/CameraVC0706" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"CameraVC0706/example/simple_snap/simple_snap.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


