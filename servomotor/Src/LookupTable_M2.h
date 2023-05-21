#ifndef __LOOKUP_TABLE2__
#define __LOOKUP_TABLE2__

#define N_HALL_SENSORS 3
#define TOTAL_NUMBER_OF_SEGMENTS 168
#define HALL_SENSOR_SHIFT 30000
#define SLOPE_SHIFT_RIGHT 22
#define OFFSET_SHIFT_RIGHT 8
#define ONE_REVOLUTION_STEPS 50
#define HALL_SAMPLES_PER_PRINT 8
#define WEIGHTS_PER_HALL_SENSOR 3
#define SENSOR_SEGMENT_RESOLUTION_DIV_2 27200
#define SENSOR_SEGMENT_RESOLUTION 54400
#define CALIBRATION_CAPTURE_STEP_SIZE 128 // this should be slow so that data can be captured and collected accurately


struct hall_weights_struct {
   int16_t h1[WEIGHTS_PER_HALL_SENSOR];
   int16_t h2[WEIGHTS_PER_HALL_SENSOR];
   int16_t h3[WEIGHTS_PER_HALL_SENSOR];
};

#define HALL_WEIGHTS_INITIALIZER { \
    { 21626,   3139,   9380}, \
    { 32280,  -5828,   3132}, \
    { 23755,   2862,  -7221}, \
}

#endif

