// Hao added it

#pragma once

#include <vector>
#include <string>
#include "ImageSourceEngine.h"

namespace InfiniTAM
{
	namespace Engine
	{
		class ImageFileEngine : public ImageSourceEngine
		{
		private:
			std::vector<std::string> depthFilelist;
			std::vector<std::string> rgbFilelist;
			Vector2i imageSize_d, imageSize_rgb;
			USHORT nDepthMinReliableDistance;
			USHORT nDepthMaxDistance;
			bool colorAvailable, depthAvailable;
			const char *depthDataPath;
			const char *rgbDataPath;
			int currentFrame;

		public:
			ImageFileEngine(const char *calibFilename, const char *depthDataPath, const char *rgbDataPath);
			~ImageFileEngine();

			bool hasMoreImages(void);
			void getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage);
			Vector2i getDepthImageSize(void);
			Vector2i getRGBImageSize(void);
		};
	}
}

