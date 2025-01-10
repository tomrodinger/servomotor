# ------------------------------------------------------------------------
# Makefile for building the firmware code + simulator natively on macOS
# using Clang and SDL2
# ------------------------------------------------------------------------

# Compiler + flags
CC = clang
CFLAGS = -Wall -g -O2 -std=gnu11 \
         -DPRODUCT_NAME_M3 -DMOTOR_SIMULATION \
         -DAT5833 -DSOFTWARE_COMPATIBILITY_CODE=3

# SDL-related flags
SDL_CFLAGS = $(shell sdl2-config --cflags)
SDL_LIBS = $(shell sdl2-config --libs) -lSDL2_ttf

# Includes
INCLUDES = -I. \
           -I../firmware/Src \
           -I../common_source_files

# Source files
FIRMWARE_SRCS = $(wildcard ../firmware/Src/*.c)
COMMON_SRCS = $(wildcard ../common_source_files/*.c)

# Filter out assembly files, system_stm32g0xx.c, ADC.c, and unique_id.c from common sources
FIRMWARE_SRCS := $(filter-out %.s %ADC.c, $(FIRMWARE_SRCS))
COMMON_SRCS := $(filter-out %.s %system_stm32g0xx.c %unique_id.c, $(COMMON_SRCS))

# Local source files
LOCAL_SRCS = \
    servo_simulator.c \
    stm32g0xx_hal.c \
    gpio.c \
    motor_hal.c \
    unique_id.c \
    settings_sim.c \
    ADC.c

# All sources and objects
ALL_SRCS = $(FIRMWARE_SRCS) $(COMMON_SRCS) $(LOCAL_SRCS)
ALL_OBJS = $(ALL_SRCS:.c=.o)

TARGET = servo_simulator

# ------------------------------------------------------------------------
# Rules
# ------------------------------------------------------------------------

all: clean $(TARGET)

$(TARGET): $(ALL_OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(SDL_LIBS)

%.o: %.c
	$(CC) $(CFLAGS) $(SDL_CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(ALL_OBJS) $(TARGET)

.PHONY: all clean
