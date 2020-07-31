#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "win_serial.h"
#include "Lidar_driver.h"
#include <iostream>
#include <fstream>

using namespace std;
#define ASSEMBLE_ANGLE 0
bool ret = false;
Serial serial;
int main(int argc, const char *argv[])
{
	ret = serial.open("COM7", 115200);
	serial.setDTR(true);
	serial.flushInput();

	float laser_angle = 0.0;
	printf("laser_ydlidar_s2 angle=%f\n",laser_angle);

	int prev_angle = 0.0, angle_count = 0, raw_angle_count = 0;
	struct laser_ranges laser_ranges;
	memset(laser_ranges.ranges, 0, sizeof(laser_ranges.ranges));
	laser_ranges.raw_count = 0;
	uint8_t *buf = new uint8_t[520];
	if (buf == NULL)
	{
		printf("new buffer failed ,no space");
		return -1;
	}
	int valid_in_buf;
	uint8_t simpleNum;
	float scanFrequenceHz = 0.0;
	uint16_t FirstSampleAngle, LastSampleAngle;
	float IntervalSampleAngle = 0, IntervalSampleAngle_LastPackage = 0;

	fstream file("lidar.txt",ios::out);

	while (ret)
	{
		valid_in_buf = 0;
		memset(buf,0,sizeof(buf));
		while (valid_in_buf < 2)
		{
			valid_in_buf += serial.read((uint8_t*)buf + valid_in_buf, 2 - valid_in_buf);
		}
		//now 2 bytes in buffer ,check header (0x55aa,LSB)
		int wasted = 0;
		while ((buf[0] != 0xaa) || (buf[1] != 0x55)) {
			buf[0] = buf[1];
			serial.read(buf + 1, 1);
			wasted++;
		}

		if (wasted > 0)
		{
			//printf("wasted:%d\n",wasted);
		}
		//now header leader ok ,read remaining head bytes(crc)
		while (valid_in_buf < 8) {
			valid_in_buf += serial.read(buf + valid_in_buf, 8 - valid_in_buf);

		}

		//angle crc error
		if ((!buf[4] & 0x01) || (!(buf[6] & 0x01))) {
			printf("angle crc fail\n");
			continue;
		}

		//now read crc
		while (valid_in_buf < 10) {
			valid_in_buf += serial.read(buf + valid_in_buf, 10 - valid_in_buf);
		}

		//now header ok, read the simple data
		simpleNum = buf[3];
		while (valid_in_buf < 10 + simpleNum * 3) {
			valid_in_buf += serial.read(buf + valid_in_buf, (10 + simpleNum * 3) - valid_in_buf);
		}

		/*now check frame crc*/
		uint16_t crcXor = buf[8] | (buf[9] << 8);// CS
		uint16_t FSA = buf[4] | (buf[5] << 8);	//
		uint16_t LSA = buf[6] | (buf[7] << 8);
		uint16_t CT_LSN = buf[2] | (buf[3] <<8);
		uint16_t PH = buf[0] | (buf[1] << 8); // PH
		uint8_t CT = buf[2];
		crcXor ^= PH;
		crcXor ^= FSA;
		//crcXor ^= CT | (simpleNum << 8); //CT+LSN
		crcXor ^= LSA; //LSA
		crcXor ^= CT_LSN;
		for (int i = 0; i < simpleNum; i++) {
			crcXor ^= buf[10 + i * 3];
			crcXor ^= ((buf[12 + i * 3] << 8) | buf[11 + i * 3]);
		}
		if (crcXor != 0)
		{
			printf("lidar S2 checksum failed\n");
			continue;
		}
		else {
			//printf("lidar S2 checksum successful\n");
		}
		FirstSampleAngle = FSA >> 1;
		LastSampleAngle = LSA >> 1;
		/*Start (zero) packet*/
		if (CT & 0x01) {
			scanFrequenceHz = (float)((CT & 0xFE) >> 1) / 10.f; //Hz
			printf("scanFrequenceHz:%f\n", scanFrequenceHz);
		}

		if (simpleNum == 1) {
			IntervalSampleAngle = 0;
		}
		else {
			if (LastSampleAngle < FirstSampleAngle) {
				if ((FirstSampleAngle > 270 * 64) && (LastSampleAngle < 90 * 64)) {
					IntervalSampleAngle = (float)((360 * 64 + LastSampleAngle - FirstSampleAngle) / ((simpleNum - 1) * 1.0));
					IntervalSampleAngle_LastPackage = IntervalSampleAngle;
				}
				else {
					IntervalSampleAngle = IntervalSampleAngle_LastPackage;
				}
			}
			else {
				IntervalSampleAngle = (float)((LastSampleAngle - FirstSampleAngle) / ((simpleNum - 1) * 1.0));
				IntervalSampleAngle_LastPackage = IntervalSampleAngle;
			}
		}
		//parse data
		for (int i = 0; i < simpleNum; i++)
		{
			uint16_t angle_q6, rawDistance, distance_q2;
			float fangle, range;
			int angle;
			uint16_t raw_Instensity,intensity;
			int32_t AngleCorrectForDistance;
			/****Distance****/
			rawDistance = *((uint16_t*)&buf[11 + i * 3]);
			distance_q2 = (rawDistance & 0xfffc) >> 2;
			/********Intensity**********/
			raw_Instensity = *((uint16_t *)&buf[10 + i * 3]);
			intensity = ((raw_Instensity & 0x300)>>8)*256+ (raw_Instensity&0xFF);
			//printf("Intensity:%2x\n", intensity);
			AngleCorrectForDistance = 0;
			if (true) {
				if (distance_q2) {
					AngleCorrectForDistance = (int32_t)(((atan(((21.8 * (155.3 - distance_q2)) / 155.3) / distance_q2))*180.0 / 3.1415) * 64.0);

				}
				if ((FirstSampleAngle + IntervalSampleAngle * i + AngleCorrectForDistance) < 0) {
					angle_q6 = (uint16_t)(FirstSampleAngle + IntervalSampleAngle * i + AngleCorrectForDistance + 360 * 64);

				}
				else {
					if ((FirstSampleAngle + IntervalSampleAngle * i + AngleCorrectForDistance) > 360 * 64) {
						angle_q6 = (uint16_t)(FirstSampleAngle + IntervalSampleAngle * i + AngleCorrectForDistance - 360 * 64);
					}
					else {
						angle_q6 = (uint16_t)(FirstSampleAngle + IntervalSampleAngle * i + AngleCorrectForDistance);
					}
				}
				fangle = (float)angle_q6 / 64.0 + ASSEMBLE_ANGLE + laser_angle;
				//printf("fangle:%f\n", fangle);
				std::cout << fangle << endl;
				angle = (int)fangle;
				range = (float)distance_q2 / 1000.f;  //m
				if (intensity == 1023) {
					printf("sunlight noise\n");
				}
				if (intensity == 1022) {
					printf("filter glass noise\n");
				}
				if (range < 0.12f || range > 8.0f) {
					range = 0;
				}

				while (angle < 0) angle += 360;
				while (angle >= 360) angle -= 360;
				
				if ((abs(angle - prev_angle) > 100) && (angle_count > 312)) {
					//SetRanges(laser_ranges, start);
					/*std::cout << "scanFreq/a/pa/ac(" << scanFrequenceHz << "-" << 3000.0 / (float)raw_angle_count << "Hz,"
						<< angle << "-" << prev_angle << "," << raw_angle_count << "-"
						<< angle_count << ")";*/
					raw_angle_count = 0;
					angle_count = 0;
					memset(laser_ranges.ranges, 0, sizeof(laser_ranges.ranges));
					laser_ranges.raw_count = 0;
				}

				raw_angle_count++;
				if (angle != prev_angle) {
					
					prev_angle = angle;
					laser_ranges.ranges[angle] = range;
					angle_count++;
					/*
					if (0 < angle && angle <= 359) {
						if((intensity != 1022) && (intensity != 1023))
						file << intensity <<endl;
					}
					else {						
						file << endl;
					}
					*/
				}
				else {
					angle_count = 0;
				}
				if (range > 0.0 && laser_ranges.raw_count < 360) {
					laser_ranges.raw_angle[laser_ranges.raw_count] = fangle;
					laser_ranges.raw_dist[laser_ranges.raw_count] = range;
					laser_ranges.raw_Intensity[laser_ranges.raw_count] = intensity;
					laser_ranges.raw_count++;
					std::cout <<"laser_rangs.raw_count="<< laser_ranges.raw_count << endl;
				}
				else {
					laser_ranges.raw_count = 0;
				}
			}
			
			//printf("Lidar S2 Intensity:%d\n",intensity);
		}
		//end for parse data

	}

	file.close();
	delete[]buf;
	return 0;
}

bool ChargeDeviceFeatureIdentify(struct laser_ranges laser_data) {
	
	for (int i = 0; i < 100; ++i) {
		
	}
	return true;
}