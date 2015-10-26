#include "ITMRasterization.h"

using namespace ITMLib::Utils;

ITMRasterization::ITMRasterization(ITMRGBDCalib *calib, Vector2i imageSize){
	this->calib = calib;
	this->imageSize = imageSize;
	depthMap = (float *)malloc(imageSize.x * imageSize.y * sizeof(float));
	normalMap = (Vector3f *)malloc(imageSize.x * imageSize.y * sizeof(Vector3f));
	memset(depthMap, 0, imageSize.x * imageSize.y * sizeof(float));
	memset(normalMap, 0, imageSize.x * imageSize.y * sizeof(Vector3f));
}

ITMRasterization::~ITMRasterization(){
	free(depthMap);
	free(normalMap);
	depthMap = NULL;
	normalMap = NULL;
}

// get rendered depth image
void ITMRasterization::getDepthImage(float *&depthMap){
	depthMap = this->depthMap;
}

void ITMRasterization::getNormals(Vector3f *&normalMap){
	normalMap = this->normalMap;
}

// get size of image
void ITMRasterization::getImageSize(Vector2i &imageSize){
	imageSize = this->imageSize;
}

float ITMRasterization::min3(const float &a, const float &b, const float &c)
{
	return min(a, min(b, c));
}

float ITMRasterization::max3(const float &a, const float &b, const float &c)
{
	return max(a, max(b, c));
}

float ITMRasterization::edgeFunction(const Vector2f &a, const Vector2f &b, const Vector2f &c){
	return (c[0] - a[0]) * (a[1] - b[1]) - (c[1] - a[1]) * (a[0] - b[0]);
}

void ITMRasterization::render(const ITMMesh::Triangle *triangles, const int noTriangles){
	Vector4f projParams_d = calib->intrinsics_d.projectionParamsSimple.all;
	Vector2i imgSize = imageSize;

	for (int i = 0; i < noTriangles; i++){
		Vector3f p0 = triangles[i].p0;
		Vector3f p1 = triangles[i].p1;
		Vector3f p2 = triangles[i].p2;

		Vector3f vec0 = p1 - p0;
		Vector3f vec1 = p2 - p1;
		Vector3f nor;
		nor.x = vec0.y*vec1.z - vec0.z*vec1.y;
		nor.y = vec0.z*vec1.x - vec0.x*vec1.z;
		nor.z = vec0.x*vec1.y - vec0.y*vec1.x;
		nor = -nor.normalised();

		Vector2f v0;
		Vector2f v1;
		Vector2f v2;

		v0.x = projParams_d.x * p0.x / p0.z + projParams_d.z;
		v0.y = projParams_d.y * p0.y / p0.z + projParams_d.w;
		v1.x = projParams_d.x * p1.x / p1.z + projParams_d.z;
		v1.y = projParams_d.y * p1.y / p1.z + projParams_d.w;
		v2.x = projParams_d.x * p2.x / p2.z + projParams_d.z;
		v2.y = projParams_d.y * p2.y / p2.z + projParams_d.w;
		
		float minx = min3(v0.x, v1.x, v2.x);
		float miny = min3(v0.y, v1.y, v2.y);
		float maxx = max3(v0.x, v1.x, v2.x);
		float maxy = max3(v0.y, v1.y, v2.y);

		// the triangle is out of screen
		if (maxx >= imgSize.x - 1 || minx < 0 || maxy >= imgSize.y - 1 || miny < 0) continue;

		float area = edgeFunction(v0, v1, v2);

		int x0 = (int)(std::floor(minx));
		int x1 = (int)(std::floor(maxx)) + 1;
		int y0 = (int)(std::floor(miny));
		int y1 = (int)(std::floor(maxy)) + 1;

		for (int y = y0; y <= y1; ++y) {
			for (int x = x0; x <= x1; ++x) {
				Vector2f pixelSample(x + 0.5, y + 0.5);
				float w0 = edgeFunction(v1, v2, pixelSample);
				float w1 = edgeFunction(v2, v0, pixelSample);
				float w2 = edgeFunction(v0, v1, pixelSample);

				w0 /= area;
				w1 /= area;
				w2 /= area;

				if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
					float z = p0.z * w0 + p1.z * w1 + p2.z * w2;

					if (depthMap[y * imgSize.x + x]==0 || z < depthMap[y * imgSize.x + x]) {
						depthMap[y * imgSize.x + x] = z;
						normalMap[y * imgSize.x + x] = nor;
					}
				}
			}
		}
	}
}
