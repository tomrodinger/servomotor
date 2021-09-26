#ifndef __LOOKUP_TABLE2__
#define __LOOKUP_TABLE2__

#define TOTAL_NUMBER_OF_SEGMENTS 30
#define HALL_SENSOR_SHIFT 30000
#define SLOPE_SHIFT_RIGHT 22
#define OFFSET_SHIFT_RIGHT 8
#define ONE_REVOLUTION_STEPS 7
#define HALL_SAMPLES_PER_PRINT 8
#define CALIBRATION_CAPTURE_STEP_SIZE 128 // this should be slow so that data can be captured and collected accurately
#define BLEND_EXTENT_SHIFT 6

struct hall_decode_data_struct {
   uint16_t min;
   uint16_t max;
   int32_t slope;
   int32_t offset;
   uint8_t active_hall_sensor1;
   uint8_t active_hall_sensor2;
};

#define HALL_DECODE_DATA_INITIALIZER { \
   { 23906, 42896,       6514,       4503, 1, 0 }, \
   { 21923, 37080,      -8469,      38564, 0, 2 }, \
   { 24008, 34653,      11715,       7463, 2, 1 }, \
   { 23808, 42081,      -6619,      47306, 1, 0 }, \
   { 21171, 35988,       8383,      24772, 0, 2 }, \
   { 23447, 34262,     -11837,      65557, 2, 1 }, \
   { 24442, 43016,       6712,      36165, 1, 0 }, \
   { 21932, 36802,      -8372,      70639, 0, 2 }, \
   { 23712, 34348,      12138,      39429, 2, 1 }, \
   { 24040, 42216,      -6614,      79639, 1, 0 }, \
   { 21934, 36608,       8313,      56809, 0, 2 }, \
   { 23872, 34668,     -11458,      97481, 2, 1 }, \
   { 24419, 43070,       6897,      68082, 1, 0 }, \
   { 21378, 36654,      -8147,     102082, 0, 2 }, \
   { 23465, 34074,      11719,      72690, 2, 1 }, \
   { 24274, 42816,      -6890,     112702, 1, 0 }, \
   { 22189, 36858,       8251,      88968, 0, 2 }, \
   { 23899, 34190,     -11654,     129961, 2, 1 }, \
   { 24000, 43025,       6919,     100596, 1, 0 }, \
   { 21640, 36578,      -8353,     134781, 0, 2 }, \
   { 24300, 34608,      11707,     104492, 2, 1 }, \
   { 24790, 43366,      -6698,     144766, 1, 0 }, \
   { 22073, 37082,       8275,     121266, 0, 2 }, \
   { 23477, 33819,     -11743,     161815, 2, 1 }, \
   { 23649, 42198,       6446,     133958, 1, 0 }, \
   { 21883, 37154,      -8718,     167893, 0, 2 }, \
   { 24029, 34480,      11510,     136820, 2, 1 }, \
   { 24657, 42768,      -6681,     176814, 1, 0 }, \
   { 21378, 36301,       8580,     153155, 0, 2 }, \
   { 23310, 33844,     -11836,     194250, 2, 1 }, \
}

#define MAX_HALL_SEGMENT 11
#define MAX_HALL_NUMBER 0
#define HALL1_MIDPOINT 29207
#define HALL2_MIDPOINT 33471
#define HALL3_MIDPOINT 28975

#endif

