#ifndef _LIDAR_HPP_
#define _LIDAR_HPP_
#include "opencv2/opencv.hpp"
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
using namespace cv;

void Print_Lidar(Mat& Basic, sl_lidar_response_measurement_node_hq_t nodes[], int count);
void Get_LR_Point(Mat& Basic, Mat& th, int& l_x, int& l_y, int& r_x, int& r_y, int cnt, Mat stats, Mat centroid);
void Get_Direction(Mat& Basic, int l_x, int l_y, int r_x, int r_y, double& dir_angle);
void Controll_mortor(double& dir_angle, int& pwm_l, int& pwm_r);
#endif