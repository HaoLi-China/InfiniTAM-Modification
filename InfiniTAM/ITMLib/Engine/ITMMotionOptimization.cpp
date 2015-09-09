#include "ITMMotionAnalysis.h"
#include "../../../3rd_libLBFGS/lbfgs.h"
#include "../../Utils/KDtree/kdtree_search_eth.h"
#include <map>

using namespace ITMLib::Engine;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// motions information
struct MotionsData
{
	MotionsData(ITMMotionAnalysis *motionAnalysis, const ITMPointCloud *visiblePointClound, ITMFloatImage *newDepthImage);
	void updatePointsNormals();  // using new warp transformations to compute the points and normals of canonical model
	void computeDpoints(float *depth);  // compute corresponding points of depth image
	void updateAllWarpInfo(double *x);

	//float computeDataTerm(const lbfgsfloatval_t* x);
	//float computeRegTerm(const lbfgsfloatval_t* x);

	//double computeDataTerm();
	//double computeRegTerm();

	ITMMotionAnalysis* malys;

	std::vector<int> vilidNodeIndex; //Nodeinfo Indices of nodes that is on surface
	std::vector<int> visibleNodeIndex; //Visible Nodeinfo Indices

	std::vector<Vector3f> points; // n, canonical model points
	std::vector<Vector3f> normals; // n, canonical model normals
	std::vector<Vector3f> dpoints; // depth image points
	std::vector<Vector3f> vpoints; // visible points
	std::vector<Vector3f> vnormals; // visible points' normals
	std::vector<int> visibles; // n, visible nodes
	std::vector<double> x0; // 6 * n warp transformations, n represents all nodes

	std::vector<std::vector<unsigned int>> neighborhood;  // n, neighbors of each node
	double lambda;

	int depth_image_width;
	int depth_image_height;
};

MotionsData::MotionsData(ITMMotionAnalysis *motionAnalysis, const ITMPointCloud *visiblePointClound, ITMFloatImage *newDepthImage)
{
	lambda = 1.0;
	depth_image_width = newDepthImage->noDims.x;
	depth_image_height = newDepthImage->noDims.y;

	malys = motionAnalysis;

	NodeInfo *allNodeinfo = NULL;
	NodeHashEntry *entryList = NULL;
	ITMRGBDCalib *calib = NULL;
	malys->getAllNodeinfo(allNodeinfo);
	malys->getAllNodeHashEntry(entryList);
	malys->getCalib(calib);

	// get indexes of nodes that is on surface
	for (int i = 0; i < NODE_BUCKET_NUM*NODE_ENTRY_NUM_PER_BUCKET + NODE_EXCESS_LIST_SIZE; i++){
		const NodeHashEntry &hashEntry = entryList[i];
		if (hashEntry.ptr >= 0){
			vilidNodeIndex.push_back(hashEntry.ptr);
		}
	}

	Vector3f *pointSet = (Vector3f*)malloc((vilidNodeIndex.size())*sizeof(Vector3f));
	for (int i = 0; i < vilidNodeIndex.size(); i++){
		int nodeindex = vilidNodeIndex[i];
		Vector3f pt(allNodeinfo[nodeindex].dg_v.x, allNodeinfo[nodeindex].dg_v.y, allNodeinfo[nodeindex].dg_v.z);
		points.push_back(pt);
		normals.push_back(Vector3f(0, 0, 0));

		pointSet[i].x = allNodeinfo[nodeindex].dg_v.x;
		pointSet[i].y = allNodeinfo[nodeindex].dg_v.y;
		pointSet[i].z = allNodeinfo[nodeindex].dg_v.z;

		x0.push_back(allNodeinfo[nodeindex].dg_se3.rx);
		x0.push_back(allNodeinfo[nodeindex].dg_se3.ry);
		x0.push_back(allNodeinfo[nodeindex].dg_se3.rz);
		x0.push_back(allNodeinfo[nodeindex].dg_se3.tx);
		x0.push_back(allNodeinfo[nodeindex].dg_se3.ty);
		x0.push_back(allNodeinfo[nodeindex].dg_se3.tz);
	}

	KdTreeSearch_ETH kd_eth;
	kd_eth.add_vertex_set(pointSet, vilidNodeIndex.size());
	kd_eth.end();

	std::vector<unsigned int> neighbors;
	for (int i = 0; i < vilidNodeIndex.size(); i++){
		int nodeindex = vilidNodeIndex[i];
		Vector3f p = allNodeinfo[nodeindex].dg_v;

		//get neighbor points within a range of radius
		kd_eth.find_points_in_radius(p, 0.05, neighbors);
	}
	neighborhood.push_back(neighbors);

	kd_eth.begin();
	free(pointSet);
	pointSet = NULL;

	Vector4f *vpoint = visiblePointClound->locations->GetData(MEMORYDEVICE_CPU);
	Vector4f *vnormal = visiblePointClound->colours->GetData(MEMORYDEVICE_CPU);
	float *depth = newDepthImage->GetData(MEMORYDEVICE_CPU);

	std::vector<int> visibleNodeIndex;
	malys->getVisibleNodeInfo(visiblePointClound, visibleNodeIndex);

	for (int i = 0; i < vilidNodeIndex.size(); i++){
		visibles.push_back(-1);
		for (int j = 0; j < visibleNodeIndex.size(); i++){
			if (vilidNodeIndex[i] == visibleNodeIndex[j]){
				visibles[i] = j;
			}
		}
	}

	for (int i = 0; i < visiblePointClound->noTotalPoints; i++){
		if (visibleNodeIndex[i] != -1){
			vpoints.push_back(Vector3f(vpoint[i].x, vpoint[i].y, vpoint[i].z));
			vnormals.push_back(Vector3f(vnormal[i].x, vnormal[i].y, vnormal[i].z));
		}
	}

	for (int i = 0; i < visibles.size(); i++){
		if (visibles[i] != -1){
			int ind = visibles[i];
			normals[i].x = vnormals[ind].x;
			normals[i].y = vnormals[ind].y;
			normals[i].z = vnormals[ind].z;
		}
	}

	updatePointsNormals();
	computeDpoints(depth);
}

//update all warp info in nodes' info
void MotionsData::updateAllWarpInfo(double *x){
	NodeInfo *allNodeinfo = NULL;
	malys->getAllNodeinfo(allNodeinfo);

	for (int i = 0; i < vilidNodeIndex.size(); i++){
		int nodeindex = vilidNodeIndex[i];

		allNodeinfo[nodeindex].dg_se3.rx = x[6*i];
		allNodeinfo[nodeindex].dg_se3.ry = x[6 * i + 1];
		allNodeinfo[nodeindex].dg_se3.rz = x[6 * i + 2];
		allNodeinfo[nodeindex].dg_se3.tx = x[6 * i + 3];
		allNodeinfo[nodeindex].dg_se3.ty = x[6 * i + 4];
		allNodeinfo[nodeindex].dg_se3.tz = x[6 * i + 5];
	}
}

/*
double MotionsData::computeDataTerm(){
	double result = 0;

	for (int i = 0; i < dpoints.size(); i++){
		if (dpoints[i].z != -1){
			Vector3f delta(vpoints[i].x - dpoints[i].x, vpoints[i].y - dpoints[i].y, vpoints[i].z - dpoints[i].z);
			double dot_result = vnormals[i].x * delta.x + vnormals[i].y * delta.y + vnormals[i].z * delta.z;

			result += (dot_result * dot_result);
		}
		else{
			continue;
		}

		return result;
	}
}


double MotionsData::computeRegTerm(){
	double result = 0;

	for (int i = 0; i < points.size(); i++){
		Vector4f v_i(points[i].x, points[i].y, points[i].z, 1.0);
		std::vector<unsigned int> neighbors = neighborhood[i];
		Transformation tf_ic = { x0[6 * i], x0[6 * i + 1], x0[6 * i + 2], x0[6 * i + 3], x0[6 * i + 4], x0[6 * i + 5] };
		Matrix4f ic;
		malys->Transformation2Matrix4(tf_ic, ic);

		for (int j = 0; j < neighbors.size(); j++){
			unsigned int ind = neighbors[j];
			Vector4f v_j = (points[ind].x, points[ind].y, points[ind].z, 1.0);
			Transformation tf_jc = { x0[6 * ind], x0[6 * ind + 1], x0[6 * ind + 2], x0[6 * ind + 3], x0[6 * ind + 4], x0[6 * ind + 5] };
			Matrix4f jc;
			malys->Transformation2Matrix4(tf_jc, jc);

			double squared_distance = (v_i.x - v_j.x)*(v_i.x - v_j.x) + (v_i.y - v_j.y)*(v_i.y - v_j.y) + (v_i.z - v_j.z)*(v_i.z - v_j.z);

			Vector4f delta = v_j*ic - v_j*jc;
			double result_tem = (delta.x*delta.x + delta.y*delta.y + delta.z*delta.z + delta.w*delta.w) * 1.0 / squared_distance;
			result += result_tem;
		}
	}

	return result;
} */

void MotionsData::updatePointsNormals()
{
	ITMRGBDCalib *calib = NULL;
	malys->getCalib(calib);

	for (int i = 0; i < points.size(); i++){
		//Vector4f pt_tem(points[i].x, points[i].y, points[i].z, 1.0);
		//Vector4f pt = pt_tem * mtf;

		//points[i].x = pt.x;
		//points[i].y = pt.y;
		//points[i].z = pt.z;

		if (visibles[i] != -1){
			Transformation tf = { x0[6 * i], x0[6 * i + 1], x0[6 * i + 2], x0[6 * i + 3], x0[6 * i + 4], x0[6 * i + 5] };
			Matrix4f mtf;
			malys->Transformation2Matrix4(tf, mtf);

			Vector4f vpt_tem(vpoints[visibles[i]].x, vpoints[visibles[i]].y, vpoints[visibles[i]].z, 1.0);
			Vector4f vpt = vpt_tem * mtf;
			Vector4f vn_tem(vpoints[visibles[i]].x, vpoints[visibles[i]].y, vpoints[visibles[i]].z, 1.0);
			Vector4f vn = vpt_tem * mtf;

			vpoints[visibles[i]].x = vpt.x;
			vpoints[visibles[i]].y = vpt.y;
			vpoints[visibles[i]].z = vpt.z;

			vnormals[visibles[i]].x = vn.x;
			vnormals[visibles[i]].y = vn.y;
			vnormals[visibles[i]].z = vn.z;
		}
	}
}

void MotionsData::computeDpoints(float *depth)
{
	dpoints.clear();

	ITMRGBDCalib *calib = NULL;
	malys->getCalib(calib);

	Vector4f projParams_d = calib->intrinsics_d.projectionParamsSimple.all;

	for (int i = 0; i < vpoints.size(); i++){
		Vector4f vpt = vpoints[i];

		Vector2f pt_image;
		pt_image.x = projParams_d.x * vpt.x / vpt.z + projParams_d.z;
		pt_image.y = projParams_d.y * vpt.y / vpt.z + projParams_d.w;

		int x = (int)(pt_image.x + 0.5f);
		int y = (int)(pt_image.y + 0.5f);

		Vector3f dpt;

		if (!((pt_image.x < 1) || (pt_image.x > depth_image_width - 2) || (pt_image.y < 1) || (pt_image.y > depth_image_height - 2))){
			dpt.z = depth[x + y * depth_image_width];
			dpt.x = dpt.z * ((float(x) - projParams_d.z) / projParams_d.x);
			dpt.y = dpt.z * ((float(y) - projParams_d.w) / projParams_d.y);
		}
		else{
			dpt.z = -1;
		}

		dpoints.push_back(dpt);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// non-linear optimization functions
static lbfgsfloatval_t motions_evaluate(
	void *data,
	const lbfgsfloatval_t *x,
	lbfgsfloatval_t *g,
	const int N,
	const lbfgsfloatval_t step
	)
{
	MotionsData* d = (MotionsData*)data;
	const std::vector<Vector3f>& points = d->points;
	const std::vector<Vector3f>& normals = d->normals;
	const std::vector<Vector3f>& dpoints = d->dpoints;
	const std::vector<double>& x0 = d->x0;
	const std::vector<int>& visibles = d->visibles;
	const std::vector<std::vector<unsigned int>>& neighborhood = d->neighborhood;
	ITMMotionAnalysis* malys  = d->malys;
	float lambda = d->lambda;
	//////////////////////////////////////////////////////////////////////////
	// initialize
	lbfgsfloatval_t f = 0;
	for (int i = 0; i < N; ++i) {
		g[i] = 0;
	}

	// data term
	for (int i = 0; i < N / 6; i++) {
		if (visibles[i] == -1)
			continue;
		
		// warp transformation of i
		Transformation tf = { x[6 * i], x[6 * i + 1], x[6 * i + 2], x[6 * i + 3], x[6 * i + 4], x[6 * i + 5] };
		Matrix4f mtf;
		malys->Transformation2Matrix4(tf, mtf);

		// sum of data term
		Vector3f vi = points[i];
		Vector3f ni = normals[i];
		Vector3f mvi = mtf * points[i];
		Vector3f mni = mtf * normals[i];
		Vector3f dvi = dpoints[visibles[i]];

		Vector3f delta_mvi_dvi(mvi.x - dvi.x, mvi.y - dvi.y, mvi.z - dvi.z);
		double dot1 = mni.x * delta_mvi_dvi.x + mni.y * delta_mvi_dvi.y + mni.z * delta_mvi_dvi.z;
		f += dot1 * dot1;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// gradient 
		// 2 * (ni.(vi - vi'))
		double dot2 = dot1;
		dot2 *= 2.0f;

		// tx, ty, tz gradient
		g[i * 6] += dot2 * (mvi.x + mni.x - dvi.x);
		g[i * 6 + 1] += dot2 * (mvi.y + mni.y - dvi.y);
		g[i * 6 + 2] += dot2 * (mvi.z + mni.z - dvi.z);

		 

		float chi = std::cos(x[i * 6 + 3]);
		float shi = std::sin(x[i * 6 + 3]);
		float cai = std::cos(x[i * 6 + 4]);
		float sai = std::sin(x[i * 6 + 4]);
		float cbi = std::cos(x[i * 6 + 5]);
		float sbi = std::sin(x[i * 6 + 5]);

		float chi_ = - std::sin(x[i * 6 + 3]);
		float shi_ = std::cos(x[i * 6 + 3]);
		float cai_ = - std::sin(x[i * 6 + 4]);
		float sai_ = std::cos(x[i * 6 + 4]);
		float cbi_ = - std::sin(x[i * 6 + 5]);
		float sbi_ = std::cos(x[i * 6 + 5]);

		// ry gradient
		// (mtf * ni) . (mtf * vi) - (mtf * ni) . dvi gradient
		g[i * 6 + 3] += (mvi.x - dvi.x) * (chi_ * cai * ni.x + (shi_ * sbi - chi_ * sai * cbi) * ni.y + (chi_ * sai * sbi + shi_ * cbi) * ni.z);
		g[i * 6 + 3] += mni.x * (chi_ * cai * vi.x + (shi_ * sbi - chi_ * sai * cbi) * vi.y + (chi_ * sai * sbi + shi_ * cbi) * vi.z);
		g[i * 6 + 3] += (mvi.z - dvi.z) * (-shi_ * cai * ni.x + (shi_ * sai * cbi + chi_ * sbi) * ni.y + (-shi_ * sai * sbi + chi_ * cbi) * ni.z);
		g[i * 6 + 3] += mni.z * (-shi_ * cai * vi.x + (shi_ * sai * cbi + chi_ * sbi) * vi.y + (-shi_ * sai * sbi + chi_ * cbi) * vi.z);		
		// 2 * (ni.(vi - vi')) * ((mtf * ni) . (mtf * vi) - (mtf * ni) . dvi)'ry
		g[i * 6 + 3] *= dot2;

		// rz gradient
		// (mtf * ni) . (mtf * vi) - (mtf * ni) . dvi gradient
		g[i * 6 + 4] += (mvi.x - dvi.x) * (chi * cai_ * ni.x + (- chi * sai_ * cbi) * ni.y + (chi * sai_ * sbi) * ni.z);
		g[i * 6 + 4] += mni.x * (chi * cai_ * vi.x + (-chi * sai_ * cbi) * vi.y + (chi * sai_ * sbi) * vi.z);
		g[i * 6 + 4] += (mvi.y - dvi.y) * (sai_ * ni.x + cai_ * cbi * ni.y  + (-cai_ * sbi) * ni.z);
		g[i * 6 + 4] += mni.y * (sai_ * vi.x + cai_ * cbi * vi.y + (-cai_ * sbi) * vi.z);
		g[i * 6 + 4] += (mvi.z - dvi.z) * ((-shi * cai_)* ni.x + shi * sai_ * cbi * ni.y + (-shi * sai_ * sbi) * ni.z);
		g[i * 6 + 4] += mni.z * ((-shi * cai_)* vi.x + shi * sai_ * cbi * vi.y + (-shi * sai_ * sbi) * vi.z);
		// 2 * (ni.(vi - vi')) * ((mtf * ni) . (mtf * vi) - (mtf * ni) . dvi)'rz
		g[i * 6 + 4] *= dot2;

		// rx gradient
		// (mtf * ni) . (mtf * vi) - (mtf * ni) . dvi gradient
		g[i * 6 + 5] += (mvi.x - dvi.x) * ((shi * sbi_ - chi * sai * cbi_)* ni.y + (chi * sai * sbi_ + shi * cbi_) * ni.z);
		g[i * 6 + 5] += mni.x * ((shi * sbi_ - chi * sai * cbi_)* vi.y + (chi * sai * sbi_ + shi * cbi_) * vi.z);
		g[i * 6 + 5] += (mvi.y - dvi.y) * (cai * cbi_ * ni.y + (-cai * sbi_) * ni.z);
		g[i * 6 + 5] += mni.y * (cai * cbi_ * vi.y + (-cai * sbi_) * vi.z);
		g[i * 6 + 5] += (mvi.z - dvi.z) * ((shi * sai * cbi_ + chi * cbi_) * ni.y + (-shi * sai * sbi_ + chi * cbi_) * ni.z);
		g[i * 6 + 5] += mni.z * ((shi * sai * cbi_ + chi * cbi_) * vi.y + (-shi * sai * sbi_ + chi * cbi_) * vi.z);
		// 2 * (ni.(vi - vi')) * ((mtf * ni) . (mtf * vi) - (mtf * ni) . dvi)'rx
		g[i * 6 + 5] *= dot2;
	}

	// reg term
	for (int i = 0; i < N / 6; i++){
		const std::vector<unsigned int>& neighbors = neighborhood[i];
		Transformation tf_ic = { x[6 * i], x[6 * i + 1], x[6 * i + 2], x[6 * i + 3], x[6 * i + 4], x[6 * i + 5] };
		Matrix4f ic;
		malys->Transformation2Matrix4(tf_ic, ic);

		//compute each sub term
		for (int j = 0; j < neighbors.size(); j++){
			unsigned int ind = neighbors[j];
			Transformation tf_jc = { x[6 * ind], x[6 * ind + 1], x[6 * ind + 2], x[6 * ind + 3], x[6 * ind + 4], x[6 * ind + 5] };
			Matrix4f jc;
			malys->Transformation2Matrix4(tf_jc, jc);

			Vector3f vi = points[i];
			Vector3f vj = points[ind];
			Vector3f mvij = ic * vj;
			Vector3f mvjj = jc * vj;

			// sum of reg term
			Vector3f vij = vi - vj;
			float squared_distance = vij.x * vij.x + vij.y * vij.y + vij.z * vij.z;
			Vector3f delta = mvij - mvjj;
			double result_tem = (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) / squared_distance;
			f += lambda * result_tem;

			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// gradient 
			float chi = std::cos(x[i * 6 + 3]);
			float shi = std::sin(x[i * 6 + 3]);
			float cai = std::cos(x[i * 6 + 4]);
			float sai = std::sin(x[i * 6 + 4]);
			float cbi = std::cos(x[i * 6 + 5]);
			float sbi = std::sin(x[i * 6 + 5]);

			float chi_ = -std::sin(x[i * 6 + 3]);
			float shi_ = std::cos(x[i * 6 + 3]);
			float cai_ = -std::sin(x[i * 6 + 4]);
			float sai_ = std::cos(x[i * 6 + 4]);
			float cbi_ = -std::sin(x[i * 6 + 5]);
			float sbi_ = std::cos(x[i * 6 + 5]);

			// i, tx, ty, tz
			g[i * 6] += lambda * 2.0f * (mvij.x - mvjj.x) * 1.0f / squared_distance;
			g[i * 6 + 1] += lambda * 2.0f * (mvij.y - mvjj.y) * 1.0f / squared_distance;
			g[i * 6 + 2] += lambda * 2.0f * (mvij.z - mvjj.z) * 1.0f / squared_distance;

			// i, ry, rz, rx gradient
			// ry
			g[i * 6 + 3] += (lambda / squared_distance) * 2.0f * (mvij.x - mvjj.x) * (chi_ * cai * vj.x + (shi_ * sbi - chi_ * sai * sbi) * vj.y + (chi_ * sai * sbi + shi_ * cbi) * vj.z);
			g[i * 6 + 3] += (lambda / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((-shi_ * cai) * vj.x + (shi_ * sai * cbi + chi_ * sbi) * vj.y + (-shi_ * sai * sbi + chi_ * cbi) * vj.z);

			// rz
			g[i * 6 + 4] += (lambda / squared_distance) * 2.0f * (mvij.x - mvjj.x) * ((chi * cai_) * vj.x + (-chi * sai_ * cbi) * vj.y + (chi * sai_ * sbi) * vj.z);
			g[i * 6 + 4] += (lambda / squared_distance) * 2.0f * (mvij.y - mvjj.y) * (sai_* vj.x + (cai_ * cbi) * vj.y + (-cai_ * sbi) * vj.z);
			g[i * 6 + 4] += (lambda / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((shi * sai * cbi_) * vj.y + (-shi * sai * sbi_ + chi * cbi_) * vj.z);

			// rx
			g[i * 6 + 5] += (lambda / squared_distance) * 2.0f * (mvij.x - mvjj.x) * ((shi * sbi_ - chi * sai * cbi_) * vj.y + (chi * sai * sbi_ + shi * cbi_) * vj.z);
			g[i * 6 + 5] += (lambda / squared_distance) * 2.0f * (mvij.y - mvjj.y) * ((cai * cbi_)* vj.y + (-cai * sbi_) * vj.z);
			g[i * 6 + 5] += (lambda / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((shi * sai * cbi_ + chi * sbi_) * vj.y + (-shi * sai * sbi_ + chi * cbi_) * vj.z);

			// j, tx, ty, tz
			g[ind * 6] += lambda * 2.0f * (mvij.x - mvjj.x) * (-1.0f) / squared_distance;
			g[ind * 6 + 1] += lambda * 2.0f * (mvij.y - mvjj.y) * (-1.0f) / squared_distance;
			g[ind * 6 + 2] += lambda * 2.0f * (mvij.z - mvjj.z) * (-1.0f) / squared_distance;

			// j, ry, rz, rx gradient
			// ry
			g[ind * 6 + 3] += (-lambda / squared_distance) * 2.0f * (mvij.x - mvjj.x) * (chi_ * cai * vj.x + (shi_ * sbi - chi_ * sai * sbi) * vj.y + (chi_ * sai * sbi + shi_ * cbi) * vj.z);
			g[ind * 6 + 3] += (-lambda / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((-shi_ * cai) * vj.x + (shi_ * sai * cbi + chi_ * sbi) * vj.y + (-shi_ * sai * sbi + chi_ * cbi) * vj.z);

			// rz
			g[ind * 6 + 4] += (-lambda / squared_distance) * 2.0f * (mvij.x - mvjj.x) * ((chi * cai_) * vj.x + (-chi * sai_ * cbi) * vj.y + (chi * sai_ * sbi) * vj.z);
			g[ind * 6 + 4] += (-lambda / squared_distance) * 2.0f * (mvij.y - mvjj.y) * (sai_* vj.x + (cai_ * cbi) * vj.y + (-cai_ * sbi) * vj.z);
			g[ind * 6 + 4] += (-lambda / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((shi * sai * cbi_) * vj.y + (-shi * sai * sbi_ + chi * cbi_) * vj.z);

			// rx
			g[ind * 6 + 5] += (-lambda / squared_distance) * 2.0f * (mvij.x - mvjj.x) * ((shi * sbi_ - chi * sai * cbi_) * vj.y + (chi * sai * sbi_ + shi * cbi_) * vj.z);
			g[ind * 6 + 5] += (-lambda / squared_distance) * 2.0f * (mvij.y - mvjj.y) * ((cai * cbi_)* vj.y + (-cai * sbi_) * vj.z);
			g[ind * 6 + 5] += (-lambda / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((shi * sai * cbi_ + chi * sbi_) * vj.y + (-shi * sai * sbi_ + chi * cbi_) * vj.z);
		}
	}

	return f;
}

static int motions_progress(
	void *data,
	const lbfgsfloatval_t *x,
	const lbfgsfloatval_t *g,
	const lbfgsfloatval_t fx,
	const lbfgsfloatval_t xnorm,
	const lbfgsfloatval_t gnorm,
	const lbfgsfloatval_t step,
	int n,
	int k,
	int ls
	)
{
	//printf("Iteration %d\n", k);
	printf("Iteration %d:  fx = %f,  xnorm = %f, gnorm = %f, step = %f\n", k, fx, xnorm, gnorm, step);
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// main optimization function
void ITMMotionAnalysis::optimizeEnergyFunction(const ITMPointCloud *visiblePointClound, ITMFloatImage *newDepthImage)
{
	MotionsData data(this, visiblePointClound, newDepthImage);
	data.lambda = 1.0;

	int n = data.x0.size();
	lbfgsfloatval_t *x = lbfgs_malloc(n);
	if (!x) {
		std::cout << "L-BFGS failed to allocate a memory block for variables." << std::endl;
		return;
	}
	/* Initialize the variables. */
	for (int i = 0; i < n; ++i) {
		x[i] = data.x0[i];
	}

	lbfgs_parameter_t param;
	/* Initialize the parameters for the L-BFGS optimization. */
	lbfgs_parameter_init(&param);
	//----------------------------------------------------------------------------

	// Start the L-BFGS optimization; this will invoke the callback functions
	// evaluate() and progress() when necessary.
	lbfgsfloatval_t fx;
	int ret = lbfgs(n, x, &fx, motions_evaluate, motions_progress, &data, &param);

	/* Report the result. */
	printf("L-BFGS optimization terminated with status code = %d\n", ret);
	printf("  fx = %f\n", fx);

	// assign new warp transformation from x
	data.updateAllWarpInfo(x);
}