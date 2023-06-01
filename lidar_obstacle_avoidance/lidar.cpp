#include "opencv2/opencv.hpp"
#include <iostream>
#include "lidar.hpp"
#include<math.h>
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"


using namespace std;
using namespace cv;

void Print_Lidar(Mat& Basic, sl_lidar_response_measurement_node_hq_t nodes[], int count)
{
    Basic = Scalar(255,255,255);
    circle(Basic, Point(Basic.rows / 2, Basic.cols / 2), 1, Scalar(0,0,0), 5);
    for(int i = 0; i < count; i++)
    {
        
    double rd_theta =  (nodes[i].angle_z_q14 * 90.f) / 16384.f;
    double d_rd_theta = rd_theta * 3.14 / 180;
    double rd_dist =  nodes[i].dist_mm_q2/4.0f / 10;

    int rd_x;
    int rd_y;

    if(rd_dist < 3) continue;
    rd_y = Basic.cols / 2 + rd_dist * cos(d_rd_theta);
    rd_x = Basic.rows / 2 - rd_dist * sin(d_rd_theta);
    rectangle(Basic, Rect(rd_x,rd_y,3,3), Scalar(0,0,255) , -1);
    }
}

void Get_LR_Point(Mat& Basic, Mat& th, int& l_x, int& l_y, int& r_x, int& r_y, int cnt, Mat stats, Mat centroid)
{
    int min_dist_r = 250;
    int min_dist_flag_r = 2000;
    int min_dist_l = 250;
    int min_dist_flag_l = 2001;
    int ob_dist;
    int flag_r = 0;
    int flag_l = 0;
    
    for(int i = 1; i < cnt; i++)
    {
        if((centroid.at<double>(i,0) - (th.cols / 2)) < 0) 
        {
            flag_l = 1;
            flag_r = 0;
        }
        else if((centroid.at<double>(i,0) - (th.cols / 2)) > 0)
        {
            flag_l = 0;
            flag_r = 1;
        }
        ob_dist = sqrt(pow(th.rows -  centroid.at<double>(i,0), 2) + pow((th.cols / 2) - centroid.at<double>(i,1), 2));
        if(flag_l)
        {
            if(ob_dist < min_dist_l)
            {
                min_dist_l = ob_dist;
                min_dist_flag_l = i;
            }
        }
        else if(flag_r)
        {
            if(ob_dist < min_dist_r)
            {
                min_dist_r = ob_dist;
                min_dist_flag_r = i;
            }
        }
    }
    int* label_l = stats.ptr<int>(min_dist_flag_l);
    int* label_r = stats.ptr<int>(min_dist_flag_r);
    int L_dist = sqrt(pow(th.rows - (label_l[0] + label_l[2]), 2) + pow((th.cols / 2) - (label_l[1] + label_l[3]), 2));
    int R_dist = sqrt(pow(th.rows - label_r[0], 2) + pow((th.cols / 2) - (label_r[1] + label_r[3]), 2));
    if(L_dist > 100 || min_dist_flag_l == 2001) 
    {
        label_l[0] = 125; label_l[2] = 0; label_l[1] = 250; label_l[3] = 0;
    }
    if(R_dist > 100 || min_dist_flag_r == 2000) 
    {
        label_r[0] = 375; label_r[2] = 0; label_r[1] = 250; label_r[3] = 0;
    }
    l_x = label_l[0] + label_l[2]; l_y = label_l[1] + label_l[3]; r_x = label_r[0]; r_y = label_r[1] + label_r[3];
    if(l_y > 250 || r_y > 250)
    {
        l_y = 250; 
        r_y = 250;
    }
    rectangle(Basic, Rect(label_l[0], label_l[1], label_l[2], label_l[3]), Scalar(0,255,0));
    rectangle(Basic, Rect(label_r[0], label_r[1], label_r[2], label_r[3]), Scalar(255,0,0));
    circle(Basic, Point(l_x, l_y), 1 ,Scalar(0,255,0), 3);
    circle(Basic, Point(r_x, r_y), 1 ,Scalar(255,0,0), 3);
    line(Basic, Point(Basic.rows / 2, Basic.cols / 2), Point(l_x, l_y), Scalar(0,255,0), 1);
    line(Basic, Point(Basic.rows / 2, Basic.cols / 2), Point(r_x, r_y), Scalar(255,0,0), 1);
    
    cout << "Point_L: " << Point(l_x, l_y) << " " << "Point_R: " << Point(r_x, r_y)<< " L_dist: " << L_dist << "   R_dist: " << R_dist << " ";
}

void Get_Direction(Mat& Basic, int l_x, int l_y, int r_x, int r_y, double& dir_angle)
{
    int center_x = Basic.cols / 2;
    int center_y = Basic.rows / 2;
    int goal_x;
    int goal_y;

    double d_l_angle = atan2(center_y - l_y, center_x - l_x);
    double l_angle = d_l_angle * 180 / 3.14;
    double d_r_angle = atan2(center_y - r_y, center_x - r_x);
    double r_angle = d_r_angle * 180 / 3.14;
    double goal_dir = l_angle + ((r_angle - l_angle) / 2);
    double d_goal_dir = (goal_dir + 90) * 3.14 / 180;
    dir_angle = 90 - goal_dir;
    goal_y = Basic.cols / 2 + 100 * cos(d_goal_dir);
    goal_x = Basic.rows / 2 - 100 * sin(d_goal_dir);

    arrowedLine(Basic, Point(Basic.rows / 2, Basic.cols / 2), Point(goal_x, goal_y), Scalar(153,0,102), 1);
    cout << /*"L_angle: " << l_angle << "  R_angle: " << r_angle <<  "  Goal_dir: " << gaol_dir << */"  Direction: " << dir_angle << "  ";
}

void Controll_mortor(double& dir_angle, int& pwm_l, int& pwm_r)
{
    int l_flag = 0;
    int r_flag = 0;

    if(dir_angle > 0) l_flag = 1;
    else if(dir_angle < 0) r_flag = 1;
    
    pwm_l = 100 - (80 * (int)dir_angle) / 100;
    pwm_r = 100 + (80 * (int)dir_angle) / 100;

    if(l_flag) cout << "왼쪽 " /*<< fabs(dir_angle) << "도 만큼 회전" */<< "    ";
    else if(r_flag) cout << "오른쪽 " /*<< fabs(dir_angle) << "도 만큼 회전" */<< "    ";
    cout << "pwm_l: " << pwm_l << "  pwm_r: " << pwm_r << "   ";
}
