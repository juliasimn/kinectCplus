#ifndef KINECTMODULE_H
#define KINECTMODULE_H

#include <Kinect.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <ctime>
#include <queue>
#include <thread>
#include <iostream>
#include <deque>

#include <assert.h>
#include <windows.h>
#include <vector>
#include <algorithm>
#include <objbase.h>
#include <direct.h>
#include <fstream>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;



//typedef unsigned short UINT16;
template <typename T>
void SafeRelease(T& ptr) { if (ptr != nullptr) { ptr->Release(); ptr = nullptr; } }


class MatDepth : public cv::Mat { };

class KinectData {
public:
	KinectData() { };
	KinectData(const cv::Mat &image_, const MatDepth &depth_) : image(image_), depth(depth_) { };

	
	cv::Mat image;
	MatDepth depth;
};

class KinectModule 
{

public:
	enum Modes : int { Color = 0, Depth = 1, Infrared = 2, Todos = 3 };

	//pruebas
	int name;
	int get_name() { return name; }


	

	HANDLE hStopEvent, hKinectThread, hKinectDequeue;
	WAITABLE_HANDLE hFrameEvent;
	int cont = 0;
	RGBQUAD* m_pColorRGBX=NULL;
	UINT16 *m_pDepthBuffer=NULL;
	UINT16 *m_pInfraredBuffer = NULL;

	//Display vars
	double ColorFrameWidth = 960.0;
	double ColorFrameHeight = 540.0;
	int DepthFrameWidth = 512;
	int DepthFrameHeight = 424;
	int InfraredFrameWidth = 512;
	int InfraredFrameHeight = 424;

	//Kinect vars
	IKinectSensor* sensor = nullptr;
	IMultiSourceFrameReader* reader = nullptr;

	// Record vars
	bool IsRecording = false;
	queue<RGBQUAD*> framesColorQueue;
	queue<RGBQUAD*> framesDepthQueue;
	queue<RGBQUAD*> framesInfraredQueue;

#define COLOR_PIXEL_TYPE CV_8UC4
#define DEPTH_PIXEL_TYPE CV_16UC1
#define INFRARED_PIXEL_TYPE CV_16UC1
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int        cInfraredWidth = 512;
	static const int        cInfraredHeight = 424;
	LONGLONG m_rgbTime, m_depthTime, m_infraredTime;
	cv::Size m_colorSize, m_depthSize, m_infraredSize;

	inline void TRACE(WCHAR const * const format, ...)
	{
		va_list args;
		va_start(args, format);

		WCHAR output[512];
		vswprintf_s(output, format, args);

		OutputDebugString(output);

		va_end(args);
	}

	// Funciones
	KinectModule(int mode, int session, int num_frames, string folderName, string nameShow);
	~KinectModule();
	void OpenSensorColor();
	void OpenSensorDepth();
	void OpenSensorInfrared();
	void OpenSensor();

	void KinectClose();
	void start();
	void stop();
	void KinectFrameArrivedEvent(IMultiSourceFrameArrivedEventArgs* e);
	void FrameArrived(IMultiSourceFrame *pFrame);
	void ColorFrameArrived(IColorFrameReference* frame);
	void DepthFrameArrived(IDepthFrameReference* frame);
	void InfraredFrameArrived(IInfraredFrameReference* frame);
	void ProcessThreadInternal();
	void StartDequeue();

private:

	string sessionFileName = "kinect.lock";
	string imageFileName = "frame";
	string folderName = ""; // time_t now = time(0); // convert now to string form char* dt = ctime(&now);
	string directorio_imagenes = "C:/Users/jsc.TIP/Desktop/Tracker_RealTime_CambioResolucion/sapkltfappdemo-5258c9724bb0/alvarogm84-sapkltfappdemo-5258c9724bb0/GUI_PKLTFilter_Tracker2/imagenes/";
	string nameGuardar = "";
	
	// Record vars
	bool initilize = false;
	int framesColorRecordedCount = 0;
	int framesDepthRecordedCount = 0;
	int framesInfraredRecordedCount = 0;

	//General vars
	string sessionId = "";
	int framesReadCount = 0;
	int framesQueued = 0;
	int kinectMode = -1;
	int numFramesTotal = 0;

	int num_frames_color_ex = 0;
	int num_frames_depth_ex = 0;
	int num_frames_infrared_ex = 0;


	//auxiliares
	int contColor = 0;
	int contDepth = 0;
	int contInfrared = 0;
	
};

#endif // KINECTMODULE_H