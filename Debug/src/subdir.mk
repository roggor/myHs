################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/cli.cpp \
../src/myHs.cpp \
../src/protocol.cpp \
../src/stanowiska.cpp 

OBJS += \
./src/cli.o \
./src/myHs.o \
./src/protocol.o \
./src/stanowiska.o 

CPP_DEPS += \
./src/cli.d \
./src/myHs.d \
./src/protocol.d \
./src/stanowiska.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -D_REENTRANT -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


