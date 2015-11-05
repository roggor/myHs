################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/cli.cpp \
../src/myHs.cpp \
../src/myjnia.cpp \
../src/protoCol.cpp \
../src/protoDev.cpp \
../src/protoDevR05.cpp \
../src/protoDevR06.cpp 

OBJS += \
./src/cli.o \
./src/myHs.o \
./src/myjnia.o \
./src/protoCol.o \
./src/protoDev.o \
./src/protoDevR05.o \
./src/protoDevR06.o 

CPP_DEPS += \
./src/cli.d \
./src/myHs.d \
./src/myjnia.d \
./src/protoCol.d \
./src/protoDev.d \
./src/protoDevR05.d \
./src/protoDevR06.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


