/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "opencv2/opencv.hpp"
#include <iostream>
#include<math.h>
#include "lidar.hpp"
#include "dxl.hpp"
using namespace cv;
using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

void print_usage(int argc, const char * argv[])
{
    printf("Simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s \n"
           "Usage:\n"
           " For serial channel %s --channel --serial <com port> [baudrate]\n"
           "The baudrate is 115200(for A2) or 256000(for A3).\n"
		   " For udp channel %s --channel --udp <ipaddr> [port NO.]\n"
           "The LPX default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
           , "SL_LIDAR_SDK_VERSION", argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
	//const char * opt_is_channel = "--channel"; 
	//const char * opt_channel = "--serial";
    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc \
    insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.168 \
    port=8001 sync=false";
    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc \
    insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.168 \
    port=8002 sync=false";

    VideoWriter writer1(dst1, 0, (double)30, cv::Size(500, 500), true);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}
    VideoWriter writer2(dst2, 0, (double)30, cv::Size(500, 250), false);
    if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}

    const char * opt_channel_param_first = "/dev/ttyUSB1";
	sl_u32         opt_channel_param_second = 115200; // sl_u32: sl -> Slamtec의 약자로, Slamtec사의 SDK에서 사용하는 자료형 접두어
    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_result     op_result;
	int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;

    Dxl mt;
    Mat Lidar(Size(500,500),CV_8UC3, Scalar(255,255,255));
    Mat th;
    Mat labels;
    Mat stats;
    Mat centroid;
    int cnt = 0;
    int cnt_lr = 0;
    int l_x, l_y, r_x, r_y;
    double dir_angle = 0;
    int pwm_l = 0;
    int pwm_r = 0;
    char ch;
    bool mode=false;

    double diff;
    struct timeval start_time,end_time;

	bool useArgcBaudrate = false;

    IChannel* _channel;

    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n", "SL_LIDAR_SDK_VERSION");

    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT){
        if(useArgcBaudrate){
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);

                if (SL_IS_OK(op_result)) 
                {
	                connectSuccess = true;
                }
                else{
                    delete drv;
					drv = NULL;
                }
            }
        }
        else{
            size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
			for(size_t i = 0; i < baudRateArraySize; ++i)
			{
				_channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);

                    if (SL_IS_OK(op_result)) 
                    {
	                    connectSuccess = true;
                        break;
                    }
                    else{
                        delete drv;
					    drv = NULL;
                    }
                }
			}
        }
    }
    else if(opt_channel_type == CHANNEL_TYPE_UDP){
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);

            if (SL_IS_OK(op_result)) 
            {
	            connectSuccess = true;
            }
            else{
                delete drv;
				drv = NULL;
            }
        }
    }


    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
			(fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
				, opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
				, opt_channel_param_first));
		
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }
    //printf("health ok\n");
    signal(SIGINT, ctrlc);
    
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
    {
        drv->stop();    
    }
    // drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);

    ////////////////
    //drv->stop();

    // fetech result and print it out...
    mt.dxl_open();
    while (1) {
        gettimeofday(&start_time,NULL);
        sl_lidar_response_measurement_node_hq_t nodes[8192]; // sl_lidar_cmd.h에 존재하는 struct: flag,angle_z_q14, dist_mm_q2, quality 값을 가지고 있음
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count); // 0~360도 스캔 데이터를 가져옴

        if (SL_IS_OK(op_result)) { // #define SL_IS_OK(x)    ( ((x) & SL_RESULT_FAIL_BIT) == 0 ), #define SL_RESULT_FAIL_BIT  (sl_result)0x80000000
            drv->ascendScanData(nodes, count); // 각도에 따라 스캔 데이터 값 상승(각도 상승시켜서 순서대로 나타냄)
            cnt = (int)count;
            // cout << "count: " << cnt << "   ";
            for (int pos = 0; pos < (int)count ; pos++) {
                /*printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", // #define SL_LIDAR_RESP_HQ_FLAG_SYNCBIT (0x1<<0)
                    (nodes[pos].angle_z_q14 * 90.f) / 16384.f, // 1z = 0.5pi= 90deg, l6384: imu센서와 같은 센서의 감도 값, 쿼터니언과 연관
                    nodes[pos].dist_mm_q2/4.0f, //  라이다에서 측정한 거리를 2배 정밀도로 표현한 값으로, 4로 나누어서 mm 단위의 값으로 변환
                    nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);*/ // #define SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT   2 //remove the last two bits and then make quality from 0-63 to 0-255 // (신호)강도 값
                // theta.at<double>(0,pos) = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                // dist.at<double>(0,pos) = nodes[pos].dist_mm_q2/4.0f;   
            }
            Print_Lidar(Lidar, nodes, cnt);
            th = Lidar(Rect(0,0,500, 255)).clone();
            inRange(th, Scalar(0,0,224), Scalar(0,0,255), th);
            morphologyEx(th, th, MORPH_CLOSE, Mat(), Point(-1,-1), 4);
            cnt_lr = connectedComponentsWithStats(th, labels, stats, centroid);
            Get_LR_Point(Lidar, th, l_x, l_y, r_x, r_y, cnt_lr, stats, centroid);
            Get_Direction(Lidar, l_x, l_y, r_x, r_y, dir_angle);
            Controll_mortor(dir_angle, pwm_l, pwm_r);
            writer1 << Lidar;
            writer2 << th;
            if(mode) mt.dxl_set_velocity(pwm_l, -pwm_r);
            if(mt.kbhit())
            { 
                ch = mt.getch();
                if(ch == 'q') 
                {
                    mt.dxl_set_velocity(0, 0);
                    break;
                }
                else if(ch == 's') mode = true;			
            } 
        }
        gettimeofday(&end_time,NULL);
        diff = end_time.tv_sec + end_time.tv_usec / 1000000.0 - start_time.tv_sec - start_time.tv_usec / 1000000.0;
        cout << "소요 시간: " << diff << "  " << endl;
        if (ctrl_c_pressed){ 
            break;
        }
    }
    mt.dxl_close();

    drv->stop(); // scan stop
	delay(200);
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
    {
        drv->setMotorSpeed(0);
        printf("scan finished\n");
    }
        
    // done!
on_finished:
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}

