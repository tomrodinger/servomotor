#ifndef __HALL_SENSOR_CONSTANTS__
#define __HALL_SENSOR_CONSTANTS__

#define N_HALL_SENSORS 3
#define TOTAL_NUMBER_OF_SEGMENTS 150
#define HALL_SENSOR_SHIFT 30000
#define SLOPE_SHIFT_RIGHT 22
#define OFFSET_SHIFT_RIGHT 8
#define HALL_SAMPLES_PER_PRINT 8
#define WEIGHTS_PER_HALL_SENSOR 3
#define SENSOR_SEGMENT_RESOLUTION 57600
#define SENSOR_SEGMENT_RESOLUTION_DIV_2 28800
#define CALIBRATION_CAPTURE_STEP_SIZE 128 // this should be slow so that data can be captured and collected accurately


struct hall_weights_struct {
   int16_t h1[WEIGHTS_PER_HALL_SENSOR];
   int16_t h2[WEIGHTS_PER_HALL_SENSOR];
   int16_t h3[WEIGHTS_PER_HALL_SENSOR];
};

#define HALL_WEIGHTS_INITIALIZER { \
    { 29538,   4495,  -2331}, \
    { 30151,   1345,   1019}, \
    { 26006,  -1156,   4042}, \
}

#endif

