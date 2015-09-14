#ifndef _ITM_MOTION_ANALYSIS
#define _ITM_MOTION_ANALYSIS

#include <vector>

#include "../Utils/ITMMath.h"
#include "../Utils/ITMLibSettings.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		struct Transformation{
			float tx, ty, tz;
			float ry, rz, rx;//y,z,x;heading, attitude, bank
		};

		struct NodeInfo{
			Vector3f dg_v; 
			Vector3f dg_w;
			Transformation dg_se3;
		};

		struct NodeHashEntry
		{
			Vector3f pos;
			int offset;
			int ptr;
		};

		class ITMMotionAnalysis
		{
		public:
			ITMMotionAnalysis(const ITMRGBDCalib *calib);
			~ITMMotionAnalysis();

			void initialize(std::vector<Vector3f> &cpoints, std::vector<Vector3f> &cnormals, std::vector<bool> &visiblelist);
			void optimizeEnergyFunction(ITMFloatImage *newDepthImage);
			void getCalib(ITMRGBDCalib *&calib);
			void getAllPoints(std::vector<Vector3f> &cpoints);
			void getAllNormals(std::vector<Vector3f> &cnormals);
			void getAllTransformations(std::vector<Transformation> &ctfs);
			void setAllTransformations(const std::vector<Transformation> &ctfs);
			void getAllVisibleList(std::vector<bool> &visiblelist);
			void Transformation2Matrix4(const Transformation &tf, Matrix4f &mtf);
			void Matrix42Transformation(const Matrix4f &mtf, Transformation &tf);
			void getAllSurfacePointsTransformation(const std::vector<std::vector<Vector3f>> &cblocks_p, const std::vector<Vector3f> &cpoints, std::vector<Transformation> &tfs);

		private:
			ITMRGBDCalib *calib;
			std::vector<Vector3f> cpoints;
			std::vector<Vector3f> cnormals;
			std::vector<Transformation> ctfs;
			std::vector<bool> visiblelist;// visiblelist size = cpoints size
		};
	}
}

#endif // _ITM_MOTION_ANALYSIS