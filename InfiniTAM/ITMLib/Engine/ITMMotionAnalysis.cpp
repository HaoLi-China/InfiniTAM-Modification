#include "ITMMotionAnalysis.h"

using namespace ITMLib::Engine;

ITMMotionAnalysis::ITMMotionAnalysis(const ITMLibSettings *settings, bool useSparseNodes){
	this->useSparseNodes = useSparseNodes;
	this->warpScene = new ITMScene<NodeInfo, ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping, MEMORYDEVICE_CPU);
}

ITMMotionAnalysis::~ITMMotionAnalysis(){

}

int ITMMotionAnalysis::hashIndex(const Vector3s voxelPos, const int hashMask) {
	return ((uint)(((uint)voxelPos.x * 73856093) ^ ((uint)voxelPos.y * 19349669) ^ ((uint)voxelPos.z * 83492791)) & (uint)hashMask);
}

int ITMMotionAnalysis::FindVBIndex(const Vector3s blockPos, const ITMHashEntry *hashTable)
{
	int offsetExcess = 0;
	int hashIdx = hashIndex(blockPos, SDF_HASH_MASK);


	//check ordered list
	for (int inBucketIdx = 0; inBucketIdx < 1; inBucketIdx++)
	{
		const ITMHashEntry &hashEntry = hashTable[hashIdx + inBucketIdx];
		offsetExcess = hashEntry.offset - 1;

		if (hashEntry.ptr < 0) return -1;

		if (hashEntry.pos == blockPos && hashEntry.ptr >= 0)
			return hashIdx + inBucketIdx;
	}

	//check excess list
	while (offsetExcess >= 0)
	{
		const ITMHashEntry &hashEntry = hashTable[SDF_BUCKET_NUM + offsetExcess];

		if (hashEntry.pos == blockPos && hashEntry.ptr >= 0)
			return SDF_BUCKET_NUM + offsetExcess;
		offsetExcess = hashEntry.offset - 1;
	}

	return -1;
}

double ITMMotionAnalysis::computeDataTerm(ITMScene<NodeInfo, ITMVoxelIndex> *warpScene, ITMPointCloud &visiblePointClound, ITMFloatImage *newDepthImage){

}

double ITMMotionAnalysis::computeRegularizationTerm(ITMScene<NodeInfo, ITMVoxelIndex> *warpScene){

}

double ITMMotionAnalysis::computeTukeyPenalty(Vector3f &n_u, Vector3f &v_u, Vector3f &vl_uw){

}

double ITMMotionAnalysis::computeHuberPenalty(Matrix4f &T_ic1, Vector3f &dg_v1, Matrix4f &T_ic2, Vector3f &dg_v2){

}

void ITMMotionAnalysis::Transformation2Matrix4(Transformation &tf, Matrix4f &mtf){

}

void ITMMotionAnalysis::Matrix42Transformation(Matrix4f &mtf, Transformation &tf){

}

void ITMMotionAnalysis::getVisibleNodeInfo(ITMScene<NodeInfo, ITMVoxelIndex> *warpScene, ITMPointCloud &visiblePointClound, NodeInfo *visibleNodeInfo){
	float voxelSize = 0.125f;
	float blockSizeWorld = warpScene->sceneParams->voxelSize*SDF_BLOCK_SIZE; // = 0.005*8;
	Vector4f *vpoint = visiblePointClound.locations->GetData(MEMORYDEVICE_CPU);

	NodeInfo *nodeinfo = warpScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry *hashTable = warpScene->index.getIndexData();

	for (int i = 0; i < visiblePointClound.noTotalPoints; i++){
		Vector3f pt;
		pt.x = vpoint[i].x;
		pt.y = vpoint[i].y;
		pt.z = vpoint[i].z;
		

		Vector3s blockPos;
		blockPos.x = floor(pt.x / blockSizeWorld);
		blockPos.y = floor(pt.y / blockSizeWorld);
		blockPos.z = floor(pt.z / blockSizeWorld);

		Vector3s voxelPoseInBlock;
		voxelPoseInBlock.x = floor((pt.x / blockSizeWorld - blockPos.x) / voxelSize);
		voxelPoseInBlock.y = floor((pt.y / blockSizeWorld - blockPos.y) / voxelSize);
		voxelPoseInBlock.z = floor((pt.z / blockSizeWorld - blockPos.z) / voxelSize);

		int lineIndex = voxelPoseInBlock.x + voxelPoseInBlock.y*SDF_BLOCK_SIZE + voxelPoseInBlock.z*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;

		int res = FindVBIndex(blockPos, hashTable);

		if (res != -1){
			const ITMHashEntry &hashEntry = hashTable[res];
			visibleNodeInfo[i] = nodeinfo[(hashEntry.ptr * SDF_BLOCK_SIZE3) + lineIndex];
		}
		else{
			vpoint[i].z = -1;
		}
	}

}