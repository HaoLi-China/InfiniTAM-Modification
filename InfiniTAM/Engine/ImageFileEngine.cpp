// Hao added it
#include <fstream>
#include "ImageFileEngine.h"
#include "../Utils/FileUtils.h"

using namespace InfiniTAM::Engine;

ImageFileEngine::ImageFileEngine(const char *calibFilename, const char *depthDataPath, const char *rgbDataPath) : ImageSourceEngine(calibFilename)
{
	if (depthDataPath == NULL)
	{
		depthAvailable = false;
		printf("No depth images\n");
		return;
	}

	this->depthDataPath = depthDataPath;
	this->rgbDataPath = rgbDataPath;

	if (rgbDataPath == NULL)
	{
		colorAvailable = false;
	}

	imageSize_d = Vector2i(512, 424);
	//imageSize_rgb = Vector2i(1920, 1080);
	imageSize_rgb = Vector2i(640, 360);

	nDepthMinReliableDistance = 500;
	nDepthMaxDistance = USHRT_MAX;

	this->calib.intrinsics_d.SetFrom(366.685, 366.685, 256.52, 208.1, 640, 480);
	this->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
	this->calib.disparityCalib.params = Vector2f(1.0f / 1000.0f, 0.0f);

	colorAvailable = true;
	depthAvailable = true;

	GetAllFiles(this->depthDataPath, depthFilelist);
	GetAllFiles(this->rgbDataPath, rgbFilelist);

	currentFrame = 0;
}

ImageFileEngine::~ImageFileEngine()
{
}

void ImageFileEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	if (currentFrame>=depthFilelist.size()){
		return;
	}

	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	if (colorAvailable)
	{
		std::ifstream infile(rgbFilelist[currentFrame], std::ios::binary);
		if (!infile)
		{
			printf("open error!\n");
			return;
		}
		
		int rgb_width;
		int rgb_height;
		char ch;
		infile >> rgb_width >> rgb_height;
		infile.read((char*)&ch, sizeof(char));

		Vector3u *rgb_buffer = new Vector3u[rgb_width*rgb_height];

		infile.read((char*)rgb_buffer, rgb_width*rgb_height*sizeof(Vector3u));

		infile.close();

		for (int i = 0; i < rgb_width*rgb_height; i++)
		{
			rgb[i].x = rgb_buffer[i].x;
			rgb[i].y = rgb_buffer[i].y;
			rgb[i].z = rgb_buffer[i].z;
			rgb[i].w = 255;
		}

		delete[]rgb_buffer;
	}
	else memset(rgb, 0, rgbImage->dataSize * sizeof(Vector4u));

	short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
	if (depthAvailable)
	{
		std::ifstream infile(depthFilelist[currentFrame], std::ios::binary);
		if (!infile)
		{
			printf("open error!\n");
			return;
		}

		int depth_width;
		int depth_height;
		char ch;
		infile >> depth_width >> depth_height;
		infile.read((char*)&ch, sizeof(char));

		ushort *depth_buffer = new ushort[depth_width*depth_height];

		infile.read((char*)depth_buffer, depth_width*depth_height*sizeof(ushort));

		infile.close();

		for (int i = 0; i < depth_width*depth_height; i++)
		{
			ushort depthPix = depth_buffer[i];
			depth[i] = (depthPix >= nDepthMinReliableDistance) && (depthPix <= nDepthMaxDistance) ? (short)depthPix : -1;
		}
		delete[]depth_buffer;
	}
	else memset(depth, 0, rawDepthImage->dataSize * sizeof(short));

	//out->inputImageType = ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE;
	currentFrame++;

	return /*true*/;
}

bool ImageFileEngine::hasMoreImages(void) { return true; }
Vector2i ImageFileEngine::getDepthImageSize(void) { return imageSize_d; }
Vector2i ImageFileEngine::getRGBImageSize(void) { return imageSize_rgb; }


