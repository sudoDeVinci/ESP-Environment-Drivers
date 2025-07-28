APP_NAME := tests
ARDUINO_LIBS := AUnit
EPOXY_DUINO_DIR ?= ../EpoxyDuino
AUNIT_DIR ?= ../AUnit

# Include parent directory for our headers
CPPFLAGS += -I..

# Define EPOXY_DUINO for conditional compilation - ADD THIS LINE
CPPFLAGS += -DEPOXY_DUINO

# Add AUnit library
CPPFLAGS += -I$(AUNIT_DIR)/src

# Add ALL your source files here
OBJS += ../I2CManager.o
OBJS += ../I2CSensor.o

# Include all test files
OBJS += tests.o
OBJS += sensorManager.o
OBJS += sensor.o
OBJS += mpu.o
OBJS += sht.o

include $(EPOXY_DUINO_DIR)/EpoxyDuino.mk