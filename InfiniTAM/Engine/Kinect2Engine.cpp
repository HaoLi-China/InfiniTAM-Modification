// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "Kinect2Engine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

#ifndef COMPILE_WITHOUT_Kinect2
#include <Kinect.h>

#pragma comment(lib, "kinect20.lib")

using namespace InfiniTAM::Engine;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class Kinect2Engine::PrivateData {
	public:
	PrivateData(void) {}

	IKinectSensor* kinectSensor;
	IDepthFrameReader* depthFrameReader;
	IColorFrameReader* colorFrameReader;
};

//Hao modified it
Kinect2Engine::Kinect2Engine(const char *calibFilename) : ImageSourceEngine(calibFilename)
{
	imageSize_d = Vector2i(512, 424);
	imageSize_rgb = Vector2i(1920, 1080);
	//imageSize_rgb = Vector2i(640, 480);

	nDepthMinReliableDistance = 500;
	nDepthMaxDistance = USHRT_MAX;
	
	data = new PrivateData();

	// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[imageSize_rgb.x * imageSize_rgb.y];

	colorAvailable = true;

	HRESULT hr;
	
	depthAvailable = true;

	hr = GetDefaultKinectSensor(&data->kinectSensor);
	if (FAILED(hr))
	{
		depthAvailable = false;
		printf("Kinect2: Failed to initialise depth camera\n");
		return;
	}

	if (data->kinectSensor)
	{
		IDepthFrameSource* pDepthFrameSource = NULL;
		IColorFrameSource* pColorFrameSource = NULL;

		hr = data->kinectSensor->Open();

		if (SUCCEEDED(hr))
			hr = data->kinectSensor->get_DepthFrameSource(&pDepthFrameSource);

		if (SUCCEEDED(hr))
			hr = pDepthFrameSource->OpenReader(&data->depthFrameReader);

		if (SUCCEEDED(hr))
			hr = data->kinectSensor->get_ColorFrameSource(&pColorFrameSource);

		if (SUCCEEDED(hr))
			hr = pColorFrameSource->OpenReader(&data->colorFrameReader);

		SafeRelease(pColorFrameSource);
		SafeRelease(pDepthFrameSource);

		this->calib.intrinsics_d.SetFrom(366.685, 366.685, 256.52, 208.1, 640, 480);
	}

	if (!data->kinectSensor || FAILED(hr))
	{
		depthAvailable = false;
		printf("Kinect2: No ready Kinect 2 sensor found\n");
		return;
	}
}

Kinect2Engine::~Kinect2Engine()
{
	SafeRelease(data->depthFrameReader);

	if (data->kinectSensor) data->kinectSensor->Close();

	SafeRelease(data->kinectSensor);

	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}
}

//Hao modified it
void Kinect2Engine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	if (colorAvailable)
	{
		IColorFrame* pColorFrame = NULL;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		RGBQUAD *c_pBuffer = NULL;

		HRESULT hr = data->colorFrameReader->AcquireLatestFrame(&pColorFrame);

		if (SUCCEEDED(hr))
		{
			if (SUCCEEDED(hr))
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);

			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&c_pBuffer));
				}
				else if (m_pColorRGBX)
				{
					c_pBuffer = m_pColorRGBX;
					nBufferSize = imageSize_rgb.x * imageSize_rgb.y * sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(c_pBuffer), ColorImageFormat_Bgra);
				}
				else
				{
					hr = E_FAIL;
				}
			}

			if (SUCCEEDED(hr) && c_pBuffer)
			{
				for (int i = 0; i < imageSize_rgb.x * imageSize_rgb.y; i++)
				{
					Vector4u newPix; 
					RGBQUAD oldPix = c_pBuffer[i];
					newPix.x = oldPix.rgbRed; 
					newPix.y = oldPix.rgbGreen; 
					newPix.z = oldPix.rgbBlue; 
					newPix.w = 255;
					rgb[i] = newPix;
				}
			}
		}
		SafeRelease(pColorFrame);
	}
	else memset(rgb, 0, rgbImage->dataSize * sizeof(Vector4u));

	short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
	if (depthAvailable)
	{
		IDepthFrame* pDepthFrame = NULL;
		UINT16 *d_pBuffer = NULL;
		UINT nBufferSize = 0;

		HRESULT hr = data->depthFrameReader->AcquireLatestFrame(&pDepthFrame);

		if (SUCCEEDED(hr))
		{
			if (SUCCEEDED(hr))
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &d_pBuffer);

			if (SUCCEEDED(hr) && d_pBuffer)
			{
				for (int i = 0; i < imageSize_d.x * imageSize_d.y; i++)
				{
					ushort depthPix = d_pBuffer[i];
					depth[i] = (depthPix >= nDepthMinReliableDistance) && (depthPix <= nDepthMaxDistance) ? (short)depthPix : -1;
				}
			}
		}

		SafeRelease(pDepthFrame);
	}
	else memset(depth, 0, rawDepthImage->dataSize * sizeof(short));

	//out->inputImageType = ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE;

	return /*true*/;
}

bool Kinect2Engine::hasMoreImages(void) { return true; }
Vector2i Kinect2Engine::getDepthImageSize(void) { return imageSize_d; }
Vector2i Kinect2Engine::getRGBImageSize(void) { return imageSize_rgb; }

#else

using namespace InfiniTAM::Engine;

Kinect2Engine::Kinect2Engine(const char *calibFilename) : ImageSourceEngine(calibFilename)
{
	printf("compiled without Kinect 2 support\n");
}
Kinect2Engine::~Kinect2Engine()
{}
void Kinect2Engine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ return; }
bool Kinect2Engine::hasMoreImages(void)
{ return false; }
Vector2i Kinect2Engine::getDepthImageSize(void)
{ return Vector2i(0,0); }
Vector2i Kinect2Engine::getRGBImageSize(void)
{ return Vector2i(0,0); }

#endif

