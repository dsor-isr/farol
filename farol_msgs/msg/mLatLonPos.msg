Header header
int8 mode  # -1 = invalid 0 = autonomous 1=DGPS 2=floatRTK 3=fixedRTK
float64 utc_time #In seconds since the begining of the day
int8 satellites
float64 latitude
float64 longitude
float64 altitude
float64 course
float64 speed_over_ground