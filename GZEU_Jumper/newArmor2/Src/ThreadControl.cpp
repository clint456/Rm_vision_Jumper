#include "../include/ThreadControl.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "CameraApi.h" //相机SDK的API头文件
#include <stdio.h>
#include <iostream>
#define TEST_CAM

Uart Serial;
using namespace cv;
unsigned char           * g_pRgbBuffer;     //处理后数据缓存区

void processer::getUartData() {

	Serial.Open("/dev/ttyUSB0", B115200, 0, false, 10);
	
}
#ifdef TEST_CAM
void processer::frameLoop() 
{
    
    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    tSdkImageResolution     pImageResolution;
    int                     iDisplayFrames = 600;
// IplImage *iplImage = NULL;
    int                     channel=3;
    int                    SetAeState;
    int                    SetExposure;
     BOOL AEstate = FALSE;
    double fps;
    double FP[10];  // 用于存放帧率的字符串

    double t = 0;
    
     CameraSdkInit(1);
     //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
	printf("state = %d\n", iStatus);

	printf("count = %d\n", iCameraCounts);
    //没有连接设备
    // if(iCameraCounts==0){
    //     return -1;
    // }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //初始化失败
	printf("state = %d\n", iStatus);
    // if(iStatus!=CAMERA_STATUS_SUCCESS){
    //     return -1;
    // }
     //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    // 设置相机分辨率
    CameraGetImageResolution(hCamera, &pImageResolution);

    pImageResolution.iIndex      = 0xFF;
    pImageResolution.iWidthFOV   = 640;
    pImageResolution.iHeightFOV  = 480;
    pImageResolution.iWidth      = 640;
    pImageResolution.iHeight     = 480;
    pImageResolution.iHOffsetFOV = static_cast<int>((640 - 640) * 0.5);
    pImageResolution.iVOffsetFOV = static_cast<int>((480 - 480) * 0.5);

    CameraSetImageResolution(hCamera, &pImageResolution);

      // 设置曝光时间
    CameraGetAeState(hCamera, &AEstate);
    SetAeState =  CameraSetAeState(hCamera, FALSE);
    printf("auto %d",SetAeState);
    SetExposure = CameraSetExposureTime(hCamera, 1000);
    printf("set %d",SetExposure);
    // 设置颜色增益
    CameraSetGain(hCamera, 145, 130, 105);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);

    string cameraMatrixFilename="/home/gs/GS/newArmor2/intrinsics_large.yml";
    armorDetector detector(cameraMatrixFilename);

    detector.enemy = BLUE;   //在这里设定[地方]enemy颜色

    // DvpCam cam(DvpCam::Original,2000,0,false);
    // cam.start_stream();
    // cam.get_info();
     //cam.loadMatrix(cameraMatrixFilename);

    
	Mat src;
    while (true) 
    {
       t = (double)cv::getTickCount();
      if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) ==
          CAMERA_STATUS_SUCCESS) {
        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

        cv::Mat src(cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                    sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8
                        ? CV_8UC1
                        : CV_8UC3,  // This represeents "Image data type"channel
                                    // selection.
                    g_pRgbBuffer);

        flip(src, src, -1);
       
        detector.run(src);
     

        // circle(detector.srcImage,Point(661,508),4,Scalar(0,255,0),6);
        circle(
            detector.srcImage,
            Point(detector.mat.at<double>(0, 2), detector.mat.at<double>(1, 2)),
            4, Scalar(0, 255, 0), 6);
        if (!detector.armors.empty()) {
          Serial.sendToMCU(1, (short)((detector.targetArmor.angle.y) * 100),
                           (short)(detector.targetArmor.angle.x * 100),
                           detector.targetArmor.distance);

          putText(detector.srcImage,
                  "distance   " + to_string(detector.targetArmor.distance),
                  Point(50, 250), 3, 1, Scalar(0, 0, 255), 2);
          putText(detector.srcImage,
                  "yaw:   " + to_string(detector.targetArmor.angle.x),
                  Point(50, 50), 1, 3, Scalar(0, 255, 0), 2);
          putText(detector.srcImage,
                  "pitch:   " + to_string(detector.targetArmor.angle.y),
                  Point(50, 150), 1, 3, Scalar(255, 0, 0), 2);

          circle(detector.srcImage, detector.targetArmor.center, 4,
                 Scalar(0, 0, 255), 3);
          imshow("srcImage", detector.srcImage);
        } else {
          // Serial.sendToMCU(2,122,133,55);
          Serial.sendToMCU(0, 0, 0, 0);
        }

        detector.clearVector();
        //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
        //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(hCamera, pbyBuffer);

        }
    // getTickcount函数：返回从操作系统启动到当前所经过的毫秒数
    // getTickFrequency函数：返回每秒的计时周期数
    // t为该处代码执行所耗的时间,单位为秒,fps为其倒数
     t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        fps = 1.0 / t;
        putText(detector.srcImage, "FPS:   " + to_string(fps), Point(50, 350),
                1, 3, Scalar(255, 100, 100), 2);
        imshow("srcImage",detector.srcImage);
	char key = waitKey(1);
            if (key == 27) {
                break;
            } 
    }
    
    CameraUnInit(hCamera);
    //注意，现反初始化后再free
    free(g_pRgbBuffer);
    waitKey(0);
}
#endif
