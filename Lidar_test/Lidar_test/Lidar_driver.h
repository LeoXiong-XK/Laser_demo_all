#ifndef _LIDAR_DRIVER_H_
#define _LIDAR_DRIVER_H_
#pragma once
#include <stdint.h>

struct laser_ranges {
	float ranges[360];
	int raw_Intensity[360];
	int raw_count;
	float raw_angle[360];
	float raw_dist[360];
	float range_min;
	float range_max;
	float angle_min;
	float angle_max;
	float intensity_min;
	float intensity_max;
};

bool ChargerDeviceFeatureIdentify(struct laser_ranges laser_data);




#endif//end _LIDAR_DRIVER_H_