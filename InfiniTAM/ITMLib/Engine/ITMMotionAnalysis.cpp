// Hao added it
#include <math.h>

#include "ITMMotionAnalysis.h"
#include "../../Utils/KDtree/kdtree_search_eth.h"

using namespace ITMLib::Engine;

ITMMotionAnalysis::ITMMotionAnalysis(const ITMLibSettings *settings, ITMRGBDCalib *calib, bool useSparseNodes){
	this->useSparseNodes = useSparseNodes;
	this->calib = calib;
	entryList = (NodeHashEntry*)malloc((NODE_BUCKET_NUM*NODE_ENTRY_NUM_PER_BUCKET + NODE_EXCESS_LIST_SIZE)*sizeof(NodeHashEntry));
	allNodeinfo = (NodeInfo*)malloc((NODE_BUCKET_NUM*NODE_ENTRY_NUM_PER_BUCKET + NODE_EXCESS_LIST_SIZE)*sizeof(NodeInfo));

	resetHashEntry();
	resetAllNodeinfo();
}

ITMMotionAnalysis::~ITMMotionAnalysis(){
	free(entryList);
	free(allNodeinfo);

	allNodeinfo = NULL;
	entryList = NULL;
}

void ITMMotionAnalysis::resetHashEntry()
{
	int noTotalEntries = (NODE_BUCKET_NUM*NODE_ENTRY_NUM_PER_BUCKET + NODE_EXCESS_LIST_SIZE);
	memset(entryList, 0, noTotalEntries*sizeof(NodeHashEntry));
	for (int i = 0; i < noTotalEntries; i++) { 
		entryList[i].ptr = -2; 
	}
}

void ITMMotionAnalysis::resetAllNodeinfo()
{
	memset(allNodeinfo, 0, (NODE_BUCKET_NUM*NODE_ENTRY_NUM_PER_BUCKET + NODE_EXCESS_LIST_SIZE)*sizeof(NodeHashEntry));
}

int ITMMotionAnalysis::hashIndex(const Vector3f nodePos, const int hashMask) {
	return ((uint)(((uint)(nodePos.x * 73856093)) ^ ((uint)(nodePos.y * 19349669)) ^ ((uint)(nodePos.z * 83492791))) & (uint)hashMask);
}

int ITMMotionAnalysis::findNodeIndex(const Vector3f nodePos, const NodeHashEntry *hashTable)
{
	int offsetExcess = 0;
	int hashIdx = hashIndex(nodePos, NODE_HASH_MASK);

	//check ordered list
	for (int inBucketIdx = 0; inBucketIdx < NODE_ENTRY_NUM_PER_BUCKET; inBucketIdx++)
	{
		const NodeHashEntry &hashEntry = hashTable[hashIdx + inBucketIdx];
		offsetExcess = hashEntry.offset - 1;

		if (hashEntry.ptr < 0) return -1;

		if (hashEntry.pos == nodePos && hashEntry.ptr >= 0)
			return hashIdx + inBucketIdx;
	}

	//check excess list
	while (offsetExcess >= 0)
	{
		const NodeHashEntry &hashEntry = hashTable[NODE_BUCKET_NUM + offsetExcess];

		if (hashEntry.pos == nodePos && hashEntry.ptr >= 0)
			return NODE_BUCKET_NUM + offsetExcess;
		offsetExcess = hashEntry.offset - 1;
	}

	return -1;
}

double ITMMotionAnalysis::Huber(double value){
	/*if (){

	}*/
	return 0;
}

double ITMMotionAnalysis::Tukey(double value){
	return 0;
}

//compute the data term
double ITMMotionAnalysis::computeDataTerm(ITMPointCloud &visiblePointClound, ITMFloatImage *newDepthImage, ITMPointCloud &livePointClound){
	Vector4f projParams_d = calib->intrinsics_d.projectionParamsSimple.all;
	Vector4f *vpoint = visiblePointClound.locations->GetData(MEMORYDEVICE_CPU);
	Vector4f *vnormal = visiblePointClound.colours->GetData(MEMORYDEVICE_CPU);
	Vector4f *npoint = livePointClound.locations->GetData(MEMORYDEVICE_CPU);
	float *depth = newDepthImage->GetData(MEMORYDEVICE_CPU);
	double result = 0;

	std::vector<int> visibleNodeIndex;
	getVisibleNodeInfo(visiblePointClound, visibleNodeIndex);

	//compute each sub term
	for (int i = 0; i < visiblePointClound.noTotalPoints; i++){
		if (visibleNodeIndex[i] != -1){
			int vnindex = visibleNodeIndex[i];
			int ptr = entryList[vnindex].ptr;
			Transformation tf = allNodeinfo[ptr].dg_se3;
			Matrix4f mtf;
			Transformation2Matrix4(tf, mtf);
			Vector4f vpt = vpoint[i] * mtf;

			Vector2f pt_image;
			pt_image.x = projParams_d.x * vpt.x / vpt.z + projParams_d.z;
			pt_image.y = projParams_d.y * vpt.y / vpt.z + projParams_d.w;

			int x = (int)(pt_image.x + 0.5f);
			int y = (int)(pt_image.y + 0.5f);

			if (!((pt_image.x < 1) || (pt_image.x > newDepthImage->noDims.x - 2) || (pt_image.y < 1) || (pt_image.y > newDepthImage->noDims.y - 2))){
				npoint[i].z = depth[x + y * newDepthImage->noDims.x];
				if(npoint[i].z == -1){
					continue;
				}

				npoint[i].x = npoint[i].z * ((float(x) - projParams_d.z) / projParams_d.x);
				npoint[i].y = npoint[i].z * ((float(y) - projParams_d.w) / projParams_d.y);
				npoint[i].w = 1.0;
			}
			else{
				npoint[i].z = -1;
				continue;
			}

			Vector3f delta(vpt.x - npoint[i].x, vpt.y - npoint[i].y, vpt.z - npoint[i].z);
			double dot_result = vnormal[i].x * delta.x + vnormal[i].y * delta.y + vnormal[i].z * delta.z;

			result += (dot_result * dot_result);
		}
		else{
			npoint[i].z = -1;
			continue;
		}
	}

	return result;
}

//compute the regularization term
double ITMMotionAnalysis::computeRegularizationTerm(){
	std::vector<int> vilidNodeIndex;
	double result = 0;

	// get indexes of nodes that is on surface
	for (int i = 0; i < NODE_BUCKET_NUM*NODE_ENTRY_NUM_PER_BUCKET + NODE_EXCESS_LIST_SIZE; i++){
		const NodeHashEntry &hashEntry = entryList[i];
		if (hashEntry.ptr >= 0){
			vilidNodeIndex.push_back(hashEntry.ptr);
		}
	}

	// get position of nodes that is on surface
	Vector3f *pointSet = (Vector3f*)malloc((vilidNodeIndex.size())*sizeof(Vector3f));
	for (int i = 0; i < vilidNodeIndex.size(); i++){
		int nodeindex = vilidNodeIndex[i];
		pointSet[i].x = allNodeinfo[nodeindex].dg_v.x;
		pointSet[i].y = allNodeinfo[nodeindex].dg_v.y;
		pointSet[i].z = allNodeinfo[nodeindex].dg_v.z;
	}

	KdTreeSearch_ETH kd_eth;
	kd_eth.add_vertex_set(pointSet, vilidNodeIndex.size());
	kd_eth.end();

	for (int i = 0; i < vilidNodeIndex.size(); i++){
		int nodeindex = vilidNodeIndex[i];
		Matrix4f ic;
		Transformation2Matrix4(allNodeinfo[nodeindex].dg_se3, ic);

		Vector3f p = allNodeinfo[nodeindex].dg_v;

		//get neighbor points within a range of radius
		std::vector<Vector3f> neighbors;
		std::vector<double> squared_distances;
		kd_eth.find_points_in_radius(p, 0.05, neighbors, squared_distances);

		//compute each sub term
		for (int j = 0; j < neighbors.size(); j++){
			int id = findNodeIndex(neighbors[j], entryList);

			if (id!=-1){
				int ptr = entryList[id].ptr;
				Vector4f v_j;
				v_j.x = allNodeinfo[ptr].dg_v.x;
				v_j.y = allNodeinfo[ptr].dg_v.y;
				v_j.z = allNodeinfo[ptr].dg_v.z;
				v_j.w = 1.0;

				Matrix4f jc;
				Transformation2Matrix4(allNodeinfo[id].dg_se3, jc);

				Vector4f delta = v_j*ic - v_j*jc;
				double result_tem = (delta.x*delta.x + delta.y*delta.y + delta.z*delta.z + delta.w*delta.w) * 1.0 / squared_distances[j];
				result += result_tem;
			}
		}
	}

	kd_eth.begin();
	free(pointSet);
	pointSet = NULL;

	return result;
}

//double ITMMotionAnalysis::computeTukeyPenalty(Vector3f &n_u, Vector3f &v_u, Vector3f &vl_uw){
//	return 0;
//}
//
//double ITMMotionAnalysis::computeHuberPenalty(Matrix4f &T_ic1, Vector3f &dg_v1, Matrix4f &T_ic2, Vector3f &dg_v2){
//	return 0;
//}

void ITMMotionAnalysis::Transformation2Matrix4(Transformation &tf, Matrix4f &mtf){
	mtf.setIdentity();

	// Assuming the angles are in radians.
	double ch = cos(tf.rz);
	double sh = sin(tf.rz);
	double ca = cos(tf.rx);
	double sa = sin(tf.rx);
	double cb = cos(tf.ry);
	double sb = sin(tf.ry);

	mtf.m00 = ch * ca;
	mtf.m01 = sh*sb - ch*sa*cb;
	mtf.m02 = ch*sa*sb + sh*cb;
	mtf.m10 = sa;
	mtf.m11 = ca*cb;
	mtf.m12 = -ca*sb;
	mtf.m20 = -sh*ca;
	mtf.m21 = sh*sa*cb + ch*sb;
	mtf.m22 = -sh*sa*sb + ch*cb;
	mtf.m03 = tf.tx;
	mtf.m13 = tf.ty;
	mtf.m23 = tf.tz;
}

void ITMMotionAnalysis::Matrix42Transformation(Matrix4f &mtf, Transformation &tf){
	tf.tx = mtf.m03;
	tf.ty = mtf.m13;
	tf.tz = mtf.m23;

	if (mtf.m10 > 0.998) { // singularity at north pole
		tf.ry = atan2(mtf.m02, mtf.m22);
		tf.rz = PI / 2;
		tf.rx = 0;
		return;
	}
	if (mtf.m10 < -0.998) { // singularity at south pole
		tf.ry = atan2(mtf.m02, mtf.m22);
		tf.rz = PI / 2;
		tf.rx = 0;
		return;
	}
	tf.ry = atan2(-mtf.m20, mtf.m00);
	tf.rx = atan2(-mtf.m12, mtf.m11);
	tf.rz = asin(mtf.m10);
}

void ITMMotionAnalysis::getVisibleNodeInfo(ITMPointCloud &visiblePointClound, std::vector<int> &visibleNodeIndex){
	float voxelSize = 0.125f;
	Vector4f *vpoint = visiblePointClound.locations->GetData(MEMORYDEVICE_CPU);

	for (int i = 0; i < visiblePointClound.noTotalPoints; i++){
		Vector3f pt;
		pt.x = vpoint[i].x;
		pt.y = vpoint[i].y;
		pt.z = vpoint[i].z;

		Vector3f nodePos;
		nodePos.x = floor(vpoint[i].x / voxelSize + 0.5)*voxelSize;
		nodePos.y = floor(vpoint[i].y / voxelSize + 0.5)*voxelSize;
		nodePos.z = floor(vpoint[i].z / voxelSize + 0.5)*voxelSize;

		int res = findNodeIndex(nodePos, entryList);

		visibleNodeIndex.push_back(res);
	}
}