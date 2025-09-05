APP_NAME := tests
ARDUINO_LIBS := AUnit
EPOXY_DUINO_DIR ?= ../EpoxyDuino
AUNIT_DIR ?= ../AUnit

# Include parent directory for our headers
CPPFLAGS += -I..

# Define EPOXY_DUINO for conditional compilation
CPPFLAGS += -DEPOXY_DUINO

# Add AUnit library
CPPFLAGS += -I$(AUNIT_DIR)/src

# Add only.cpp source files here
OBJS += ../I2CManager.o
OBJS += ../MPU6050/MPU6050.o
OBJS += ../SHT3X/SHT31D.o

# Include all test files
OBJS += sensorManager.o
OBJS += sensor.o
OBJS += mpu.o
OBJS += sht.o

include $(EPOXY_DUINO_DIR)/EpoxyDuino.mk