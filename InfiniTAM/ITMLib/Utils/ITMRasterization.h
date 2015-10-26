#ifndef _ITM_RASTERIZATION
#define _ITM_RASTERIZATION

#include <vector>

#include "ITMLibSettings.h"
#include "../Objects/ITMMesh.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Utils
	{
		class ITMRasterization
		{
		public:
			ITMRasterization(ITMRGBDCalib *calib, Vector2i imageSize);
			~ITMRasterization();
			void render(const ITMMesh::Triangle *triangles, const int noTriangles);
			void getDepthImage(float *&depthMap);
			void getNormals(Vector3f *&normalMap);
			void getImageSize(Vector2i &imageSize);

		private:
			Vector2i imageSize;
			ITMRGBDCalib *calib;
			float *depthMap;
			Vector3f *normalMap;

			float edgeFunction(const Vector2f &a, const Vector2f &b, const Vector2f &c);
			float min3(const float &a, const float &b, const float &c);
			float max3(const float &a, const float &b, const float &c);
		};
	}
}

#endif // _ITM_RASTERIZATION