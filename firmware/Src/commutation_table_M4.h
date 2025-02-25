#ifndef __COMMUTATION_TABLE__
#define __COMMUTATION_TABLE__

// * * * DO NOT EDIT * * * Instead, edit the program that autogenerates this
// This file was autogenerated by executing: ./BLDC_sin_lookup_table.py M4
// Today's date is: Wed Nov 20 11:20:16 2024

struct three_phase_data_struct {
   uint32_t phase1;
   uint32_t phase2;
   int32_t phase1_slope;
   int32_t phase2_slope;
};

#define N_COMMUTATION_STEPS 256
#define N_COMMUTATION_SUB_STEPS 256
#define N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT 8
#define ONE_REVOLUTION_ELECTRICAL_CYCLES 50
#define ONE_REVOLUTION_MICROSTEPS 3276800
#define MAX_PHASE_VALUE 16777215

#define COMMUTATION_LOOKUP_TABLE_INITIALIZER { \
   {   8388608,          0,     205866,       2526}, \
   {   8594474,       2526,     205742,       7577}, \
   {   8800217,      10104,     205494,      12624}, \
   {   9005712,      22729,     205123,      17664}, \
   {   9210835,      40393,     204627,      22692}, \
   {   9415463,      63086,     204009,      27707}, \
   {   9619472,      90794,     203267,      32706}, \
   {   9822740,     123500,     202403,      37684}, \
   {  10025144,     161185,     201418,      42640}, \
   {  10226562,     203825,     200311,      47570}, \
   {  10426873,     251396,     199083,      52472}, \
   {  10625956,     303868,     197735,      57342}, \
   {  10823692,     361211,     196268,      62177}, \
   {  11019961,     423388,     194683,      66975}, \
   {  11214644,     490364,     192981,      71733}, \
   {  11407626,     562097,     191162,      76447}, \
   {  11598789,     638545,     189229,      81115}, \
   {  11788018,     719661,     187181,      85735}, \
   {  11975199,     805396,     185021,      90303}, \
   {  12160220,     895699,     182749,      94816}, \
   {  12342970,     990516,     180367,      99273}, \
   {  12523337,    1089789,     177876,     103669}, \
   {  12701214,    1193459,     175278,     108003}, \
   {  12876493,    1301463,     172575,     112272}, \
   {  13049068,    1413735,     169768,     116474}, \
   {  13218836,    1530209,     166858,     120605}, \
   {  13385695,    1650815,     163848,     124663}, \
   {  13549544,    1775479,     160739,     128647}, \
   {  13710284,    1904126,     157534,     132553}, \
   {  13867818,    2036680,     154233,     136379}, \
   {  14022052,    2173059,     150840,     140123}, \
   {  14172892,    2313183,     147356,     143783}, \
   {  14320249,    2456966,     143783,     147356}, \
   {  14464032,    2604323,     140123,     150840}, \
   {  14604156,    2755163,     136379,     154233}, \
   {  14740535,    2909397,     132553,     157534}, \
   {  14873089,    3066931,     128647,     160739}, \
   {  15001736,    3227671,     124663,     163848}, \
   {  15126400,    3391520,     120605,     166858}, \
   {  15247006,    3558379,     116474,     169768}, \
   {  15363480,    3728147,     112272,     172575}, \
   {  15475752,    3900722,     108003,     175278}, \
   {  15583756,    4076001,     103669,     177876}, \
   {  15687426,    4253878,      99273,     180367}, \
   {  15786699,    4434245,      94816,     182749}, \
   {  15881516,    4616995,      90303,     185021}, \
   {  15971819,    4802016,      85735,     187181}, \
   {  16057554,    4989197,      81115,     189229}, \
   {  16138670,    5178426,      76447,     191162}, \
   {  16215118,    5369589,      71733,     192981}, \
   {  16286851,    5562571,      66975,     194683}, \
   {  16353827,    5757254,      62177,     196268}, \
   {  16416004,    5953523,      57342,     197735}, \
   {  16473347,    6151259,      52472,     199083}, \
   {  16525819,    6350342,      47570,     200311}, \
   {  16573390,    6550653,      42640,     201418}, \
   {  16616030,    6752071,      37684,     202403}, \
   {  16653715,    6954475,      32706,     203267}, \
   {  16686421,    7157743,      27707,     204009}, \
   {  16714129,    7361752,      22692,     204627}, \
   {  16736822,    7566380,      17664,     205123}, \
   {  16754486,    7771503,      12624,     205494}, \
   {  16767111,    7976998,       7577,     205742}, \
   {  16774689,    8182741,       2526,     205866}, \
   {  16777215,    8388608,      -2526,     205866}, \
   {  16774689,    8594474,      -7577,     205742}, \
   {  16767111,    8800217,     -12624,     205494}, \
   {  16754486,    9005712,     -17664,     205123}, \
   {  16736822,    9210835,     -22692,     204627}, \
   {  16714129,    9415463,     -27707,     204009}, \
   {  16686421,    9619472,     -32706,     203267}, \
   {  16653715,    9822740,     -37684,     202403}, \
   {  16616030,   10025144,     -42640,     201418}, \
   {  16573390,   10226562,     -47570,     200311}, \
   {  16525819,   10426873,     -52472,     199083}, \
   {  16473347,   10625956,     -57342,     197735}, \
   {  16416004,   10823692,     -62177,     196268}, \
   {  16353827,   11019961,     -66975,     194683}, \
   {  16286851,   11214644,     -71733,     192981}, \
   {  16215118,   11407626,     -76447,     191162}, \
   {  16138670,   11598789,     -81115,     189229}, \
   {  16057554,   11788018,     -85735,     187181}, \
   {  15971819,   11975199,     -90303,     185021}, \
   {  15881516,   12160220,     -94816,     182749}, \
   {  15786699,   12342970,     -99273,     180367}, \
   {  15687426,   12523337,    -103669,     177876}, \
   {  15583756,   12701214,    -108003,     175278}, \
   {  15475752,   12876493,    -112272,     172575}, \
   {  15363480,   13049068,    -116474,     169768}, \
   {  15247006,   13218836,    -120605,     166858}, \
   {  15126400,   13385695,    -124663,     163848}, \
   {  15001736,   13549544,    -128647,     160739}, \
   {  14873089,   13710284,    -132553,     157534}, \
   {  14740535,   13867818,    -136379,     154233}, \
   {  14604156,   14022052,    -140123,     150840}, \
   {  14464032,   14172892,    -143783,     147356}, \
   {  14320249,   14320249,    -147356,     143783}, \
   {  14172892,   14464032,    -150840,     140123}, \
   {  14022052,   14604156,    -154233,     136379}, \
   {  13867818,   14740535,    -157534,     132553}, \
   {  13710284,   14873089,    -160739,     128647}, \
   {  13549544,   15001736,    -163848,     124663}, \
   {  13385695,   15126400,    -166858,     120605}, \
   {  13218836,   15247006,    -169768,     116474}, \
   {  13049068,   15363480,    -172575,     112272}, \
   {  12876493,   15475752,    -175278,     108003}, \
   {  12701214,   15583756,    -177876,     103669}, \
   {  12523337,   15687426,    -180367,      99273}, \
   {  12342970,   15786699,    -182749,      94816}, \
   {  12160220,   15881516,    -185021,      90303}, \
   {  11975199,   15971819,    -187181,      85735}, \
   {  11788018,   16057554,    -189229,      81115}, \
   {  11598789,   16138670,    -191162,      76447}, \
   {  11407626,   16215118,    -192981,      71733}, \
   {  11214644,   16286851,    -194683,      66975}, \
   {  11019961,   16353827,    -196268,      62177}, \
   {  10823692,   16416004,    -197735,      57342}, \
   {  10625956,   16473347,    -199083,      52472}, \
   {  10426873,   16525819,    -200311,      47570}, \
   {  10226562,   16573390,    -201418,      42640}, \
   {  10025144,   16616030,    -202403,      37684}, \
   {   9822740,   16653715,    -203267,      32706}, \
   {   9619472,   16686421,    -204009,      27707}, \
   {   9415463,   16714129,    -204627,      22692}, \
   {   9210835,   16736822,    -205123,      17664}, \
   {   9005712,   16754486,    -205494,      12624}, \
   {   8800217,   16767111,    -205742,       7577}, \
   {   8594474,   16774689,    -205866,       2526}, \
   {   8388608,   16777215,    -205866,      -2526}, \
   {   8182741,   16774689,    -205742,      -7577}, \
   {   7976998,   16767111,    -205494,     -12624}, \
   {   7771503,   16754486,    -205123,     -17664}, \
   {   7566380,   16736822,    -204627,     -22692}, \
   {   7361752,   16714129,    -204009,     -27707}, \
   {   7157743,   16686421,    -203267,     -32706}, \
   {   6954475,   16653715,    -202403,     -37684}, \
   {   6752071,   16616030,    -201418,     -42640}, \
   {   6550653,   16573390,    -200311,     -47570}, \
   {   6350342,   16525819,    -199083,     -52472}, \
   {   6151259,   16473347,    -197735,     -57342}, \
   {   5953523,   16416004,    -196268,     -62177}, \
   {   5757254,   16353827,    -194683,     -66975}, \
   {   5562571,   16286851,    -192981,     -71733}, \
   {   5369589,   16215118,    -191162,     -76447}, \
   {   5178426,   16138670,    -189229,     -81115}, \
   {   4989197,   16057554,    -187181,     -85735}, \
   {   4802016,   15971819,    -185021,     -90303}, \
   {   4616995,   15881516,    -182749,     -94816}, \
   {   4434245,   15786699,    -180367,     -99273}, \
   {   4253878,   15687426,    -177876,    -103669}, \
   {   4076001,   15583756,    -175278,    -108003}, \
   {   3900722,   15475752,    -172575,    -112272}, \
   {   3728147,   15363480,    -169768,    -116474}, \
   {   3558379,   15247006,    -166858,    -120605}, \
   {   3391520,   15126400,    -163848,    -124663}, \
   {   3227671,   15001736,    -160739,    -128647}, \
   {   3066931,   14873089,    -157534,    -132553}, \
   {   2909397,   14740535,    -154233,    -136379}, \
   {   2755163,   14604156,    -150840,    -140123}, \
   {   2604323,   14464032,    -147356,    -143783}, \
   {   2456966,   14320249,    -143783,    -147356}, \
   {   2313183,   14172892,    -140123,    -150840}, \
   {   2173059,   14022052,    -136379,    -154233}, \
   {   2036680,   13867818,    -132553,    -157534}, \
   {   1904126,   13710284,    -128647,    -160739}, \
   {   1775479,   13549544,    -124663,    -163848}, \
   {   1650815,   13385695,    -120605,    -166858}, \
   {   1530209,   13218836,    -116474,    -169768}, \
   {   1413735,   13049068,    -112272,    -172575}, \
   {   1301463,   12876493,    -108003,    -175278}, \
   {   1193459,   12701214,    -103669,    -177876}, \
   {   1089789,   12523337,     -99273,    -180367}, \
   {    990516,   12342970,     -94816,    -182749}, \
   {    895699,   12160220,     -90303,    -185021}, \
   {    805396,   11975199,     -85735,    -187181}, \
   {    719661,   11788018,     -81115,    -189229}, \
   {    638545,   11598789,     -76447,    -191162}, \
   {    562097,   11407626,     -71733,    -192981}, \
   {    490364,   11214644,     -66975,    -194683}, \
   {    423388,   11019961,     -62177,    -196268}, \
   {    361211,   10823692,     -57342,    -197735}, \
   {    303868,   10625956,     -52472,    -199083}, \
   {    251396,   10426873,     -47570,    -200311}, \
   {    203825,   10226562,     -42640,    -201418}, \
   {    161185,   10025144,     -37684,    -202403}, \
   {    123500,    9822740,     -32706,    -203267}, \
   {     90794,    9619472,     -27707,    -204009}, \
   {     63086,    9415463,     -22692,    -204627}, \
   {     40393,    9210835,     -17664,    -205123}, \
   {     22729,    9005712,     -12624,    -205494}, \
   {     10104,    8800217,      -7577,    -205742}, \
   {      2526,    8594474,      -2526,    -205866}, \
   {         0,    8388608,       2526,    -205866}, \
   {      2526,    8182741,       7577,    -205742}, \
   {     10104,    7976998,      12624,    -205494}, \
   {     22729,    7771503,      17664,    -205123}, \
   {     40393,    7566380,      22692,    -204627}, \
   {     63086,    7361752,      27707,    -204009}, \
   {     90794,    7157743,      32706,    -203267}, \
   {    123500,    6954475,      37684,    -202403}, \
   {    161185,    6752071,      42640,    -201418}, \
   {    203825,    6550653,      47570,    -200311}, \
   {    251396,    6350342,      52472,    -199083}, \
   {    303868,    6151259,      57342,    -197735}, \
   {    361211,    5953523,      62177,    -196268}, \
   {    423388,    5757254,      66975,    -194683}, \
   {    490364,    5562571,      71733,    -192981}, \
   {    562097,    5369589,      76447,    -191162}, \
   {    638545,    5178426,      81115,    -189229}, \
   {    719661,    4989197,      85735,    -187181}, \
   {    805396,    4802016,      90303,    -185021}, \
   {    895699,    4616995,      94816,    -182749}, \
   {    990516,    4434245,      99273,    -180367}, \
   {   1089789,    4253878,     103669,    -177876}, \
   {   1193459,    4076001,     108003,    -175278}, \
   {   1301463,    3900722,     112272,    -172575}, \
   {   1413735,    3728147,     116474,    -169768}, \
   {   1530209,    3558379,     120605,    -166858}, \
   {   1650815,    3391520,     124663,    -163848}, \
   {   1775479,    3227671,     128647,    -160739}, \
   {   1904126,    3066931,     132553,    -157534}, \
   {   2036680,    2909397,     136379,    -154233}, \
   {   2173059,    2755163,     140123,    -150840}, \
   {   2313183,    2604323,     143783,    -147356}, \
   {   2456966,    2456966,     147356,    -143783}, \
   {   2604323,    2313183,     150840,    -140123}, \
   {   2755163,    2173059,     154233,    -136379}, \
   {   2909397,    2036680,     157534,    -132553}, \
   {   3066931,    1904126,     160739,    -128647}, \
   {   3227671,    1775479,     163848,    -124663}, \
   {   3391520,    1650815,     166858,    -120605}, \
   {   3558379,    1530209,     169768,    -116474}, \
   {   3728147,    1413735,     172575,    -112272}, \
   {   3900722,    1301463,     175278,    -108003}, \
   {   4076001,    1193459,     177876,    -103669}, \
   {   4253878,    1089789,     180367,     -99273}, \
   {   4434245,     990516,     182749,     -94816}, \
   {   4616995,     895699,     185021,     -90303}, \
   {   4802016,     805396,     187181,     -85735}, \
   {   4989197,     719661,     189229,     -81115}, \
   {   5178426,     638545,     191162,     -76447}, \
   {   5369589,     562097,     192981,     -71733}, \
   {   5562571,     490364,     194683,     -66975}, \
   {   5757254,     423388,     196268,     -62177}, \
   {   5953523,     361211,     197735,     -57342}, \
   {   6151259,     303868,     199083,     -52472}, \
   {   6350342,     251396,     200311,     -47570}, \
   {   6550653,     203825,     201418,     -42640}, \
   {   6752071,     161185,     202403,     -37684}, \
   {   6954475,     123500,     203267,     -32706}, \
   {   7157743,      90794,     204009,     -27707}, \
   {   7361752,      63086,     204627,     -22692}, \
   {   7566380,      40393,     205123,     -17664}, \
   {   7771503,      22729,     205494,     -12624}, \
   {   7976998,      10104,     205742,      -7577}, \
   {   8182741,       2526,     205866,      -2526}, \
}

#endif
