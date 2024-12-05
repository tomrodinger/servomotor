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
   { 23626, 43908,       6055,       4386, 0, 1 }, \
   { 21476, 38680,      -7088,      35174, 1, 2 }, \
   { 22360, 38860,       7249,      14181, 2, 0 }, \
   { 22462, 43477,      -6452,      45862, 0, 1 }, \
   { 22438, 39274,       7186,      24589, 1, 2 }, \
   { 23549, 39604,      -7488,      58386, 2, 0 }, \
   { 22692, 43017,       6494,      36024, 0, 1 }, \
   { 21130, 38137,      -7285,      67659, 1, 2 }, \
   { 23578, 39702,       7565,      45276, 2, 0 }, \
   { 22374, 43074,      -6178,      77856, 0, 1 }, \
   { 22622, 39957,       7427,      56339, 1, 2 }, \
   { 22456, 39614,      -7190,      89702, 2, 0 }, \
   { 22192, 42228,       6262,      69245, 0, 1 }, \
   { 21652, 39342,      -7368,     100518, 1, 2 }, \
   { 23625, 40011,       7475,      77543, 2, 0 }, \
   { 23323, 42970,      -6045,     109801, 0, 1 }, \
   { 22316, 39344,       7847,      88148, 1, 2 }, \
   { 22780, 39857,      -7439,     122310, 2, 0 }, \
   { 23630, 43789,       6217,     101043, 0, 1 }, \
   { 22314, 39529,      -7448,     133207, 1, 2 }, \
   { 24231, 40660,       7562,     109553, 2, 0 }, \
   { 21850, 43066,      -5865,     141326, 0, 1 }, \
   { 21869, 38693,       7137,     122151, 1, 2 }, \
   { 23308, 40211,      -7841,     155582, 2, 0 }, \
   { 23532, 43641,       6004,     133421, 0, 1 }, \
   { 22820, 38937,      -7502,     165448, 1, 2 }, \
   { 23168, 39878,       7836,     141651, 2, 0 }, \
   { 21862, 42434,      -6276,     174312, 0, 1 }, \
   { 22604, 39513,       7113,     154093, 1, 2 }, \
   { 23304, 40152,      -7695,     188026, 2, 0 }, \
}

#define MAX_HALL_SEGMENT 19
#define MAX_HALL_NUMBER 0
#define HALL1_MIDPOINT 32855
#define HALL2_MIDPOINT 30607
#define HALL3_MIDPOINT 31500

#endif

