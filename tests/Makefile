APP_NAME := tests
ARDUINO_LIBS := AUnit
EPOXY_DUINO_DIR ?= ../EpoxyDuino
AUNIT_DIR ?= ../AUnit

# Include parent directory for our headers
CPPFLAGS += -I..

# Add AUnit library
CPPFLAGS += -I$(AUNIT_DIR)/src

# Add ALL your source files here - this is what's missing!
OBJS += ../I2CManager.o
OBJS += ../I2CSensor.o

# Include all test files
OBJS += tests.o
OBJS += sensorManager.o
OBJS += sensor.o
OBJS += mpu.o
OBJS += sht.o

include $(EPOXY_DUINO_DIR)/EpoxyDuino.mk