#ifndef _ITM_MOTION_ANALYSIS
#define _ITM_MOTION_ANALYSIS

#include "../Utils/ITMMath.h"
#include "../Objects/ITMPointCloud.h"
#include "../Objects/ITMScene.h"
#include "../Utils/ITMLibSettings.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		struct Transformation{
			float tx, ty, tz;
			float rx, ry, rz;
		};

		struct NodeInfo{
			Vector3f dg_v; 
			Vector3f dg_w;
			Transformation dg_se3;
		};

		class ITMMotionAnalysis
		{
		public:
			ITMMotionAnalysis(const ITMLibSettings *settings, bool useSparseNodes);
			~ITMMotionAnalysis();
			void optimizeEnergyFunction(ITMScene<NodeInfo, ITMVoxelIndex> *warpScene, ITMPointCloud &visiblePointClound, ITMFloatImage *newDepthImage);

		private:
			bool useSparseNodes;
			ITMScene<NodeInfo, ITMVoxelIndex> *warpScene;

			int hashIndex(const Vector3s voxelPos, const int hashMask);
			int FindVBIndex(const Vector3s blockPos, const ITMHashEntry *hashTable);
			double computeDataTerm(ITMScene<NodeInfo, ITMVoxelIndex> *warpScene, ITMPointCloud &visiblePointClound, ITMFloatImage *newDepthImage);
			double computeRegularizationTerm(ITMScene<NodeInfo, ITMVoxelIndex> *warpScene);
			double computeTukeyPenalty(Vector3f &n_u, Vector3f &v_u, Vector3f &vl_uw);
			double computeHuberPenalty(Matrix4f &T_ic1, Vector3f &dg_v1, Matrix4f &T_ic2, Vector3f &dg_v2);
			void Transformation2Matrix4(Transformation &tf, Matrix4f &mtf);
			void Matrix42Transformation(Matrix4f &mtf, Transformation &tf);
			void getVisibleNodeInfo(ITMScene<NodeInfo, ITMVoxelIndex> *warpScene, ITMPointCloud &visiblePointClound, NodeInfo *visibleNodeInfo);
		};
	}
}

#endif // _ITM_MOTION_ANALYSIS