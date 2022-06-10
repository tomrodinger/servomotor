#ifndef __LOOKUP_TABLE2__
#define __LOOKUP_TABLE2__

#define N_HALL_SENSORS 2
#define MAGNETIC_RING_POLE_PAIRS 6
#define MAGNETIC_RING_POLES (MAGNETIC_RING_POLE_PAIRS * 2)
#define TOTAL_NUMBER_OF_SEGMENTS (MAGNETIC_RING_POLES * 3)
#define HALL_SENSOR_SHIFT 30000
#define SLOPE_SHIFT_RIGHT 22
#define OFFSET_SHIFT_RIGHT 8
#define ONE_REVOLUTION_STEPS 6
#define HALL_SAMPLES_PER_PRINT 8
#define WEIGHTS_PER_HALL_SENSOR 2
#define SENSOR_SEGMENT_RESOLUTION 11520
#define SENSOR_SEGMENT_RESOLUTION_DIV_2 (SENSOR_SEGMENT_RESOLUTION / 2)
#define CALIBRATION_CAPTURE_STEP_SIZE 128 // this should be slow so that data can be captured and collected accurately

// #define HALL1_MIDLINE 38995
// #define HALL2_MIDLINE 39988
// #define HALL3_MIDLINE 38995
// #define HALL4_MIDLINE 39988

#define HALL1_MIDLINE 38755
#define HALL2_MIDLINE 37887
#define HALL3_MIDLINE 38755
#define HALL4_MIDLINE 37887

struct hall_weights_struct {
   int16_t h1[WEIGHTS_PER_HALL_SENSOR];
   int16_t h2[WEIGHTS_PER_HALL_SENSOR];
};

// #define HALL_WEIGHTS_INITIALIZER_1_2 { \
//     {  6472,    0}, \
//     {  0,   6472}, \
// }
//
// #define HALL_WEIGHTS_INITIALIZER_3_4 { \
//     {  6472,   0}, \
//     {  0,   6472}, \
// }

// #define HALL_WEIGHTS_INITIALIZER_1_2 { \
//     {  3176,   1294}, \
//     {  -903,  16349}, \
// }
//
// #define HALL_WEIGHTS_INITIALIZER_3_4 { \
//     {  3176,   1294}, \
//     {  -903,  16349}, \
// }

#define HALL_WEIGHTS_INITIALIZER_1_2 { \
    { 10989,   5103}, \
    { -4830,  11840}, \
}

#define HALL_WEIGHTS_INITIALIZER_3_4 { \
    { 10989,   5103}, \
    { -4830,  11840}, \
}

#endif
