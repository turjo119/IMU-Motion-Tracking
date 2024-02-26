#ifndef BATTERY_MONITOR
#define BATTERY_MONITOR

// Voltage Mapping function
float mapf(float x, float in_min, float in_max, float out_min,
float out_max);

float batterymonitor();

#endif