#ifndef _ITM_MOTION_ANALYSIS
#define _ITM_MOTION_ANALYSIS

#include <vector>

#include "../Utils/ITMMath.h"
#include "../Objects/ITMPointCloud.h"
#include "../Objects/ITMScene.h"
#include "../Utils/ITMLibSettings.h"

#define NODE_ENTRY_NUM_PER_BUCKET 1		// Number of entries in each Hash Bucket
#define NODE_BUCKET_NUM 0x100000		// Number of Hash Bucket, should be 2^n and bigger than SDF_LOCAL_BLOCK_NUM, SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define NODE_HASH_MASK 0xfffff			// Used for get hashing value of the bucket index,  SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define NODE_EXCESS_LIST_SIZE 0x20000	// 0x20000 Size of excess list, used to handle collisions. Also max offset (unsigned short) value.

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
			ITMMotionAnalysis(const ITMLibSettings *settings, const ITMRGBDCalib *calib, bool useSparseNodes);
			~ITMMotionAnalysis();
			void optimizeEnergyFunction(const ITMPointCloud *visiblePointClound, ITMFloatImage *newDepthImage);
			void getAllNodeinfo(NodeInfo *&nodeinfo);
			void getAllNodeHashEntry(NodeHashEntry *&entryList);
			void getCalib(ITMRGBDCalib *&calib);
			void getVisibleNodeInfo(const ITMPointCloud *visiblePointClound, std::vector<int> &visibleNodeIndex);
			void Transformation2Matrix4(const Transformation &tf, Matrix4f &mtf);
			void Matrix42Transformation(const Matrix4f &mtf, Transformation &tf);
			void setAllNodeinfo(const std::vector<Vector3f> &points);
			void getAllTransformation(const std::vector<Vector3f> &points, std::vector<Transformation> &tfs);

		private:
			bool useSparseNodes;
			ITMRGBDCalib *calib;
			NodeHashEntry *entryList;
			NodeInfo *allNodeinfo;

			int hashIndex(const Vector3f nodePos, const int hashMask);
			int findNodeIndex(const Vector3f nodePos, const NodeHashEntry *hashTable);
			void resetHashEntry();
			void resetAllNodeinfo();

			double Huber(double value);
			double Tukey(double value);
			double computeDataTerm(const ITMPointCloud *visiblePointClound, ITMFloatImage *newDepthImage, ITMPointCloud &livePointClound);
			double computeRegularizationTerm();
			//double computeTukeyPenalty(Vector3f &n_u, Vector3f &v_u, Vector3f &vl_uw);
			//double computeHuberPenalty(Matrix4f &T_ic1, Vector3f &dg_v1, Matrix4f &T_ic2, Vector3f &dg_v2);
		};
	}
}

#endif // _ITM_MOTION_ANALYSIS