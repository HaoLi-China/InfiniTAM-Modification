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
			ITMMotionAnalysis(const ITMRGBDCalib *calib, bool useControlPoints);
			~ITMMotionAnalysis();

			//const float INFLUENCE_RADIUS = sqrt(0.04*0.04*3);
			const float INFLUENCE_RADIUS = 0.1;

			void initialize(std::vector<Vector3f> &cpoints, std::vector<Vector3f> &cnormals, std::vector<bool> &visiblelist, std::vector<std::vector<unsigned int>> visibleNeighbors);
			void optimizeEnergyFunctionNlopt(ITMFloatImage *newDepthImage, const std::vector<Vector3f> &visiblePoints, const std::vector<Vector3f> &visibleNormals);
			void getCalib(ITMRGBDCalib *&calib);
			void getAllPoints(std::vector<Vector3f> &cpoints);
			void getAllNormals(std::vector<Vector3f> &cnormals);
			void getAllTransformations(std::vector<Transformation> &ctfs);
			void setAllTransformations(const std::vector<Transformation> &ctfs);
			void getAllVisibleList(std::vector<bool> &visiblelist);
			void getVisibleNeighbors(std::vector<std::vector<unsigned int>> &visibleNeighbors);
			void Transformation2Matrix4(const Transformation &tf, Matrix4f &mtf);
			void Transformation2RotTrans(const Transformation &tf, std::vector<float>& rot, std::vector<float>& trans); // get rotation and translation
			Vector3f TransformPoint(const std::vector<float>& rot, const std::vector<float>& trans, const Vector3f& point); // transform point
			Vector3f TransformNormal(const std::vector<float>& rot, const Vector3f& normal); // transform normal
			bool invTransformation(const std::vector<float>& source_rot, const std::vector<float>& source_trans, std::vector<float>& target_rot, std::vector<float>& target_trans);//inverse transformation
			
			void RotTrans2Transformation(const std::vector<float>& rot, const std::vector<float>& trans, Transformation &tf); // get transformation
			void Matrix42Transformation(const Matrix4f &mtf, Transformation &tf);
			void transformAllPoints(std::vector<Vector3f> &cpoints, std::vector<Vector3f> &cnormals);
			void inferTransformations(const std::vector<Vector3f> &cpoints, const std::vector<Transformation> &ctfs, const std::vector<Vector3f> &npoints, std::vector<Transformation> &ntfs);
			
			void updateTransformation(Transformation &step, Transformation &oldTransformation, Transformation &newTransformation);
			Vector3f interpolateBilinear(const std::vector<Vector3f> &source, const Vector2f & position, const Vector2i &imgSize);

			bool changeDpWhenIteration;
			int findDepthPointsPolicy;

		private:
			ITMRGBDCalib *calib;
			std::vector<Vector3f> cpoints;
			std::vector<Vector3f> cnormals;
			std::vector<Transformation> ctfs;
			std::vector<bool> visiblelist;// visiblelist size = cpoints size
			std::vector<std::vector<unsigned int>> visibleNeighbors;

			bool useControlPoints;

			void getNeighboorsOfEachNode(const std::vector<Vector3f> &cpoints, const float radius, std::vector<std::vector<unsigned int>> &neighbor);
		};

		struct MotionsData
		{
			MotionsData(ITMMotionAnalysis *motionAnalysis, float *depth, int depth_width, int depth_height, const std::vector<Vector3f> &visiblePoints, const std::vector<Vector3f> &visibleNormals);
			void updatePointsNormals();  // using new warp transformations to compute the points and normals of canonical model
			void computeDpoints(const float *depth, const std::vector<double>& x, std::vector<Vector3f> &cdps, std::vector<Vector3f> &dps, const float disThreshold);  // compute corresponding points of depth image
			void updateAllWarpInfo(double *x);
			void updateAllWarpInfo(const std::vector<double>& x);
			void findNeighborsInDepthMap(int x, int y, int scale, std::vector<Vector2i> &pos_s);

			ITMMotionAnalysis* malys;

			std::vector<Vector3f> points; // n, canonical model points
			std::vector<Vector3f> normals; // n, canonical model normals
			std::vector<Vector3f> dpoints; // depth image points
			std::vector<Vector3f> cdpoints; // depth image points
			std::vector<Vector3f> vpoints; // visible points
			std::vector<Vector3f> vnormals; // visible points' normals
			std::vector<int> visibles; // n, visible nodes
			std::vector<bool> livelist; // n, live nodes
			std::vector<double> x0; // 6 * n warp transformations, n represents all nodes
			std::vector<Vector3f> visiblePoints;
			std::vector<Vector3f> visibleNormals;
			std::vector<std::vector<unsigned int>> visible_neighbors;

			std::vector<std::vector<unsigned int>> neighborhood;  // n, neighbor nodes of each node
	
			float alfa;
			float beta;
			float lambda;

			int depth_image_width;
			int depth_image_height;

			float *depth;

			bool rotationOnly;
		};
	}
}

#endif // _ITM_MOTION_ANALYSIS