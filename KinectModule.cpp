#include "KinectModule.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Windows.h>
//#include <Python.h>

using namespace cv;
using namespace std;

DWORD ProcessThread(LPVOID pParam) {
	KinectModule * kinect = (KinectModule*)pParam;
	kinect->ProcessThreadInternal();
	return 0;
}

DWORD ProcessThread2(LPVOID pParam) {
	KinectModule * kinect = (KinectModule*)pParam;
	kinect->StartDequeue();
	return 0;
}


KinectModule::KinectModule(int mode, int session, int num_frames, string nombre, string nameShow) {

	std::cout << "New kinectSensor session: " <<session<<"\n";
	OutputDebugStringW(L"Llega a Kinect Module\n");
	HRESULT hr;
	hr = GetDefaultKinectSensor(&sensor);
	hr = sensor->Open();
	if (FAILED(hr)) {
		printf("Failed to find the kinect sensor\n");
		OutputDebugStringW(L"Failed to find the kinect sensor");
		exit(10);
	}
	name = 1;
	numFramesTotal = num_frames;
	kinectMode = mode;
	folderName = nombre;
	if (nameShow == "") {
		nameGuardar = "NO_NAME";
	}
	else {
		nameGuardar = nameShow;
	}



	sessionId = std::to_string(session);
	
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	m_colorSize = Size(cColorWidth, cColorHeight);
	m_depthSize = Size(cDepthWidth, cDepthHeight);
	m_infraredSize = Size(cDepthWidth, cDepthHeight);
	m_pDepthBuffer = new UINT16[cDepthWidth*cDepthHeight];
}

KinectModule::~KinectModule()
{

}

void KinectModule::start() {

	HRESULT hr = reader->SubscribeMultiSourceFrameArrived(&hFrameEvent);
	if (FAILED(hr)) {
		throw exception("Couldn't subscribe frame");
	}

	hStopEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	hKinectThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&ProcessThread, this, 0, NULL);
}

void KinectModule::stop() {
	//stop the ProcessThread
	if (hStopEvent != NULL) {
		//signal the process to stop
		SetEvent(hStopEvent);
		if (hKinectThread != NULL) {
			WaitForSingleObject(hKinectThread, INFINITE);
			CloseHandle(hKinectThread);
			hKinectThread = NULL;
		}
		CloseHandle(hStopEvent);
		hStopEvent = NULL;
		reader->UnsubscribeMultiSourceFrameArrived(hFrameEvent);
		hFrameEvent = NULL;
	}
	/*if (m_pColorRGBX) {
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}*/
}

// Funciones que abren los diferentes readers
void KinectModule::OpenSensorColor() {
	sensor->OpenMultiSourceFrameReader(
		FrameSourceTypes::FrameSourceTypes_Color,
		&reader);
}

void KinectModule::OpenSensorDepth() {
	sensor->OpenMultiSourceFrameReader(
		FrameSourceTypes::FrameSourceTypes_Depth,
		&reader);
}

void KinectModule::OpenSensorInfrared() {
	sensor->OpenMultiSourceFrameReader(
		FrameSourceTypes::FrameSourceTypes_Infrared,
		&reader);
}

void KinectModule::OpenSensor() {
	sensor->OpenMultiSourceFrameReader(
		FrameSourceTypes::FrameSourceTypes_Depth
		| FrameSourceTypes::FrameSourceTypes_Color
		| FrameSourceTypes::FrameSourceTypes_Infrared,
		&reader);
}

void KinectModule::KinectClose() {
	SafeRelease(reader);
	if (sensor != nullptr) {
		sensor->Close();
		SafeRelease(sensor);
	}
}

void KinectModule::ProcessThreadInternal() {
	HANDLE handles[] = { reinterpret_cast<HANDLE>(hFrameEvent) };
	int idx;
	bool quit = false;
	while (!quit) {
		//OutputDebugStringW(L"ProcessThereadInternal: entramos en el while \n");
		// Wait for any of the events to be signalled
		idx = WaitForMultipleObjects(1, handles, FALSE, 100);
		switch (idx) {
		case WAIT_TIMEOUT:
			continue;
		case WAIT_OBJECT_0:
			//OutputDebugStringW(L"ProcessThereadInternal: esperamos a hFrameEvent \n"); 
			IMultiSourceFrameArrivedEventArgs *pFrameArgs = nullptr;
			HRESULT hr = reader->GetMultiSourceFrameArrivedEventData(hFrameEvent, &pFrameArgs);
			//frame arrived
			KinectFrameArrivedEvent(pFrameArgs);
			pFrameArgs->Release();
			break;
		}
		if (WaitForSingleObject(hStopEvent, 1) == WAIT_OBJECT_0)
			quit = true;
	}
}

void KinectModule::KinectFrameArrivedEvent(IMultiSourceFrameArrivedEventArgs *e) {
	OutputDebugStringW(L"event\n");
	HRESULT hr;
	IMultiSourceFrameReference *pFrameReference = nullptr;
	hr = e->get_FrameReference(&pFrameReference);
	if (SUCCEEDED(hr)) {
		if (IsRecording) {
			IMultiSourceFrame *pFrame = nullptr;
			hr = pFrameReference->AcquireFrame(&pFrame);
			if (FAILED(hr)) {
				cout << "fail on AcquireFrame" << endl;
			}
			FrameArrived(pFrame);
		}
	}
	pFrameReference->Release();
	return;
}

void KinectModule::FrameArrived(IMultiSourceFrame *pFrame) {
	
	if (IsRecording && !initilize)
	{
		hKinectDequeue = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&ProcessThread2, this, 0, NULL);
		initilize = true;
	}

	HRESULT hr;
		

	IColorFrameReference* pColorFrameReference = nullptr;
	IDepthFrameReference* pDepthFrameReference = nullptr;
	IInfraredFrameReference* pInfraredFrameReference = nullptr;

	if (kinectMode == (int)Modes::Color) {
		hr = pFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
			ColorFrameArrived(pColorFrameReference);
		SafeRelease(pColorFrameReference);
	}else if (kinectMode == (int)Modes::Depth) {
		hr = pFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
			DepthFrameArrived(pDepthFrameReference);
		SafeRelease(pDepthFrameReference);
	} else if(kinectMode == (int)Modes::Infrared) {
		hr = pFrame->get_InfraredFrameReference(&pInfraredFrameReference);
		if (SUCCEEDED(hr))
			InfraredFrameArrived(pInfraredFrameReference);
		SafeRelease(pInfraredFrameReference);
	}

		
}


void KinectModule::ColorFrameArrived(IColorFrameReference* frame) {
	OutputDebugStringW(L"color\n");
	//char* str;
	IColorFrame* pColorFrame = NULL;
	HRESULT hr = frame->AcquireFrame(&pColorFrame);
	if (FAILED(hr)) {
		//cout << "Couldn't acquire color frame" << endl;
		return;
	}
	//cout << "got a color frame" << endl;
	INT64 nColorTime = 0;
	IFrameDescription* pColorFrameDescription = NULL;
	int nColorWidth = 0;
	int nColorHeight = 0;
	ColorImageFormat imageFormat = ColorImageFormat_None;
	UINT nColorBufferSize = 0;
	RGBQUAD *pColorBuffer = NULL;
	m_pColorRGBX = NULL;
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	// get color frame data
	hr = pColorFrame->get_RelativeTime(&nColorTime);
	if (SUCCEEDED(hr)) {
		hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
	}
	if (SUCCEEDED(hr)) {
		hr = pColorFrameDescription->get_Width(&nColorWidth);
	}
	if (SUCCEEDED(hr)) {
		hr = pColorFrameDescription->get_Height(&nColorHeight);
	}
	if (SUCCEEDED(hr)) {
		hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
	}
	if (SUCCEEDED(hr)) {
		if (imageFormat == ColorImageFormat_Bgra) {
			hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			Mat tmp = Mat(m_colorSize, COLOR_PIXEL_TYPE, pColorBuffer, Mat::AUTO_STEP);
			imwrite("imagen2.jpg", tmp);
		}
		else if (m_pColorRGBX) {
			pColorBuffer = m_pColorRGBX;
			nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
			hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);

			framesColorQueue.push(pColorBuffer);
		}
		else {
			hr = E_FAIL;
		}
	}
	SafeRelease(pColorFrameDescription);
	SafeRelease(pColorFrame);
}

void KinectModule::StartDequeue() {

	while (IsRecording || !framesColorQueue.empty() || !framesDepthQueue.empty() || !framesInfraredQueue.empty()) {
		if (kinectMode == (int)Modes::Color) {
			if (!framesColorQueue.empty()) {
				try {
					RGBQUAD* pColorBuffer = framesColorQueue.front();
					//framesColorQueue.pop();
					Mat tmp = Mat(m_colorSize, COLOR_PIXEL_TYPE, pColorBuffer, Mat::AUTO_STEP);
					// La primera vez creamos la carpeta
					if (contColor == 0) {
						string folder = directorio_imagenes+ folderName;
						mkdir(folder.c_str());
						string nombre = directorio_imagenes + "nombres.txt";
						fstream fs(nombre.c_str(), ios::app);
						int contador = 0;
						if (fs.fail()) {
							cerr << "El fichero no existe" << endl;
							OutputDebugStringW(L"El fichero no existe\n");
						}
						else
							OutputDebugStringW(L"El archivo ha sido modificado correctamente\n");
						fs << folder;
						fs << "\t";
						fs << nameGuardar;
						fs << "\n";
						fs.close();

					}
					std::string resultado = directorio_imagenes+folderName+"/frame" + std::to_string(contColor) + ".bmp";
					imwrite(resultado, tmp);
					contColor++;
				}
				catch (Exception e) {
					OutputDebugStringW(L"Exception\n");
				}
				framesColorQueue.pop();
			}
			
			
		}
		else if (kinectMode == (int)Modes::Depth && !framesDepthQueue.empty() ) {
			OutputDebugStringW(L"desencolamos\n");
			try {
				RGBQUAD* pInicio = framesDepthQueue.front();
				Mat tmp = Mat(m_depthSize, COLOR_PIXEL_TYPE, pInicio, Mat::AUTO_STEP);
				std::string resultado = "C:/Users/jsc.TIP/Desktop/Tracker_RealTime_CambioResolucion/sapkltfappdemo-5258c9724bb0/alvarogm84-sapkltfappdemo-5258c9724bb0/GUI_PKLTFilter_Tracker2/imagenes/imagen_DEPTH" + std::to_string(contDepth) + ".bmp";
				imwrite(resultado, tmp);
				contDepth++;
			}
			catch (Exception e) {
				OutputDebugStringW(L"Exception\n");
			}

			framesDepthQueue.pop();
		}
		else if (kinectMode == (int)Modes::Infrared && !framesInfraredQueue.empty()) {
			try {
				RGBQUAD* pInicio = framesInfraredQueue.front();
				Mat tmp = Mat(m_depthSize, COLOR_PIXEL_TYPE, pInicio, Mat::AUTO_STEP);
				std::string resultado = "C:/Users/jsc.TIP/Desktop/Tracker_RealTime_CambioResolucion/sapkltfappdemo-5258c9724bb0/alvarogm84-sapkltfappdemo-5258c9724bb0/GUI_PKLTFilter_Tracker2/imagenes/imagen_infrared" + std::to_string(contInfrared) + ".bmp";
				imwrite(resultado, tmp);
				contInfrared++;
			}
			catch (Exception e) {
				OutputDebugStringW(L"Exception\n");
			}

			framesInfraredQueue.pop();
		}

	}
}

void KinectModule::DepthFrameArrived(IDepthFrameReference* frame) {
	IDepthFrame* pDepthFrame = NULL;
	HRESULT hr = frame->AcquireFrame(&pDepthFrame);
	if (FAILED(hr))
		return;

	INT64 nDepthTime = 0;
	IFrameDescription* pDepthFrameDescription = NULL;

	int nDepthWidth = 0;
	int nDepthHeight = 0;
	
	UINT nDepthBufferSize=0;
	
	// get depth frame data
	hr = pDepthFrame->get_RelativeTime(&nDepthTime);
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
	}
	if (SUCCEEDED(hr)) {
		hr = pDepthFrameDescription->get_Width(&nDepthWidth);
	}
	if (SUCCEEDED(hr)) {
		hr = pDepthFrameDescription->get_Height(&nDepthHeight);
	}
	if (SUCCEEDED(hr)) {
		nDepthBufferSize = nDepthHeight*nDepthWidth;
		m_pDepthBuffer = new UINT16[nDepthBufferSize];
		hr = pDepthFrame->CopyFrameDataToArray(nDepthBufferSize, m_pDepthBuffer);
		USHORT nMaxDepth;
		USHORT nMinDepth;
		pDepthFrame->get_DepthMaxReliableDistance(&nMaxDepth);
		pDepthFrame->get_DepthMinReliableDistance(&nMinDepth);
		RGBQUAD* pRGBX = NULL;
		pRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
		RGBQUAD* pInicio = pRGBX;
		const UINT16* pBufferEnd = m_pDepthBuffer + (cDepthHeight*cDepthWidth);
		while (m_pDepthBuffer < pBufferEnd) {
			USHORT depth = *m_pDepthBuffer;
			BYTE insensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);
			pRGBX->rgbRed = insensity;
			pRGBX->rgbBlue = insensity;
			pRGBX->rgbGreen = insensity;
			++pRGBX;
			++m_pDepthBuffer;
		}
		framesDepthQueue.push(pInicio);

		m_pDepthBuffer = nullptr;
		pRGBX = nullptr;
		pInicio = nullptr;
	}
	SafeRelease(pDepthFrameDescription);
	SafeRelease(pDepthFrame);
}


// Esto hay que cambiarlo para que sea para infrared

void KinectModule::InfraredFrameArrived(IInfraredFrameReference* frame) {
	IInfraredFrame* pInfraredFrame = NULL;
	HRESULT hr = frame->AcquireFrame(&pInfraredFrame);
	if (FAILED(hr))
		return;
	//cout << "got a depth frame" << endl;
	INT64 nInfraredTime = 0;
	IFrameDescription* pInfraredFrameDescription = NULL;
	int nInfraredWidth = 0;
	int nInfraredHeight = 0;
	UINT nInfraredBufferSize = 0;

	// get depth frame data
	hr = pInfraredFrame->get_RelativeTime(&nInfraredTime);
	if (SUCCEEDED(hr)) {
		hr = pInfraredFrame->get_FrameDescription(&pInfraredFrameDescription);
	}
	if (SUCCEEDED(hr)) {
		hr = pInfraredFrameDescription->get_Width(&nInfraredWidth);
	}
	if (SUCCEEDED(hr)) {
		hr = pInfraredFrameDescription->get_Height(&nInfraredHeight);
	}
	if (SUCCEEDED(hr)) {

		nInfraredBufferSize = nInfraredHeight*nInfraredWidth;
		m_pInfraredBuffer = new UINT16[nInfraredBufferSize];
		hr = pInfraredFrame->CopyFrameDataToArray(nInfraredBufferSize, m_pInfraredBuffer);
		
		RGBQUAD* pRGBX = new RGBQUAD[nInfraredWidth * nInfraredHeight];
		RGBQUAD* pInicio = pRGBX;
		const UINT16* pBufferEnd = m_pInfraredBuffer + (nInfraredHeight*nInfraredWidth);
		while (m_pInfraredBuffer < pBufferEnd) {
			USHORT ir = *m_pInfraredBuffer;
			BYTE insensity = static_cast<BYTE>(ir>>8);
			pRGBX->rgbRed = insensity;
			pRGBX->rgbBlue = insensity;
			pRGBX->rgbGreen = insensity;
			++pRGBX;
			++m_pInfraredBuffer;
		}
		framesInfraredQueue.push(pInicio);
		

		m_pInfraredBuffer = nullptr;
		pRGBX = nullptr;
		pInicio = nullptr;


	}
	SafeRelease(pInfraredFrameDescription);
	SafeRelease(pInfraredFrame);

}


















