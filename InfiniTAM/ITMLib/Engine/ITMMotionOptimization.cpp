#include "ITMMotionAnalysis.h"
#include "../../../3rd_libLBFGS/lbfgs.h"
#include "../../Utils/KDtree/kdtree_search_eth.h"
#include "../../Utils/PointsIO/PointsIO.h"
#include <map>
#include "nlopt.hpp"

using namespace ITMLib::Engine;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// motions information

MotionsData::MotionsData(ITMMotionAnalysis *motionAnalysis, ITMFloatImage *newDepthImage)
{
	alfa = 0.0f;
	beta = 0.0f;
	lambda = 0.0f;

	depth_image_width = newDepthImage->noDims.x;
	depth_image_height = newDepthImage->noDims.y;

	malys = motionAnalysis;

	ITMRGBDCalib *calib = NULL;
	std::vector<Transformation> tfs;
	std::vector<bool> visiblelist;
	malys->getAllPoints(points);
	malys->getAllNormals(normals);
	malys->getAllTransformations(tfs);
	malys->getCalib(calib);
	malys->getAllVisibleList(visiblelist);

	for (int i = 0; i < tfs.size(); i++){		
		x0.push_back(tfs[i].tx);
		x0.push_back(tfs[i].ty);
		x0.push_back(tfs[i].tz);
		x0.push_back(tfs[i].ry);
		x0.push_back(tfs[i].rz);
		x0.push_back(tfs[i].rx);
	}

	int count = 0;
	for (int i = 0; i < visiblelist.size(); i++){
		if (visiblelist[i]){
			visibles.push_back(count);
			count++;
		}
		else{
			visibles.push_back(-1);
		}
	}

	for (int i = 0; i < points.size(); i++){
		if (visibles[i] != -1){
			vpoints.push_back(Vector3f(points[i].x, points[i].y, points[i].z));
			vnormals.push_back(Vector3f(normals[i].x, normals[i].y, normals[i].z));
		}
	}

	//just for debug
	//PointsIO::savePLYfile("vpoints.ply", vpoints, vnormals);
	//PointsIO::savePLYfile("vpoints.ply", vpoints, vnormals, Vector3u(255, 0, 0));

	//updatePointsNormals();
	float *depth_device = newDepthImage->GetData(MEMORYDEVICE_CUDA);
	float *depth = (float*)malloc(depth_image_width*depth_image_height * sizeof(float));
	ITMSafeCall(cudaMemcpy(depth, depth_device, (depth_image_width * depth_image_height)*sizeof(float), cudaMemcpyDeviceToHost));

	computeDpoints(depth);

	free(depth);
	depth = NULL;

	//just for debug
	//std::vector<Vector3f> dnormals;
	//PointsIO::savePLYfile("dpoints.ply", dpoints, dnormals, Vector3u(0, 0, 255));

	int valid_dpoints_num = 0;
	for (int i = 0; i < dpoints.size(); i++){
		if (dpoints[i].z != -1){
			valid_dpoints_num++;
		}
	}

	Vector3f *visiblePointSet = (Vector3f*)malloc(valid_dpoints_num*sizeof(Vector3f));
	memset(visiblePointSet, 0, valid_dpoints_num*sizeof(Vector3f));
	Vector3f *allPointSet = (Vector3f*)malloc(points.size()*sizeof(Vector3f));
	memset(allPointSet, 0, points.size()*sizeof(Vector3f));

	int id = 0;
	for (int i = 0; i < points.size(); i++){
		if (visibles[i] != -1 && dpoints[visibles[i]].z != -1){
			visiblePointSet[id].x = points[i].x;
			visiblePointSet[id].y = points[i].y;
			visiblePointSet[id].z = points[i].z;

			id++;
		}

		allPointSet[i].x = points[i].x;
		allPointSet[i].y = points[i].y;
		allPointSet[i].z = points[i].z;
	}

	KdTreeSearch_ETH kd_eth1;
	kd_eth1.add_vertex_set(visiblePointSet, valid_dpoints_num);
	kd_eth1.end();

	KdTreeSearch_ETH kd_eth2;
	kd_eth2.add_vertex_set(allPointSet, points.size());
	kd_eth2.end();

	std::vector<unsigned int> neighbors;
	for (int i = 0; i < points.size(); i++){
		Vector3f p = points[i];
		//get neighbor points within a range of radius(0.04m)
		//kd_eth1.find_points_in_radius(p, 0.0016, neighbors);
		std::vector<double> squared_distances;
		kd_eth1.find_closest_K_points(p, 3, neighbors, squared_distances);

		double sum_dis = 0;
		for (int j = 0; j < squared_distances.size(); j++){
			sum_dis += squared_distances[j];
		}

		if (sum_dis > 0.03){
			livelist.push_back(false);
			kd_eth2.find_closest_K_points(p, 4, neighbors);
		}
		else{
			livelist.push_back(true);
		}

		neighborhood.push_back(neighbors);
	}

	kd_eth1.begin();
	free(visiblePointSet);
	visiblePointSet = NULL;

	kd_eth2.begin();
	free(allPointSet);
	allPointSet = NULL;
}

//update all warp info in nodes' info
void MotionsData::updateAllWarpInfo(double *x){
	std::vector<Transformation> tfs;
	malys->getAllTransformations(tfs);

	for (int i = 0; i < tfs.size(); i++){		
		tfs[i].tx = x[6 * i];
		tfs[i].ty = x[6 * i + 1];
		tfs[i].tz = x[6 * i + 2];
		tfs[i].ry = x[6 * i + 3];
		tfs[i].rz = x[6 * i + 4];
		tfs[i].rx = x[6 * i + 5];
	}
	malys->setAllTransformations(tfs);
}

void MotionsData::updateAllWarpInfo(const std::vector<double>& x)
{
	std::vector<Transformation> tfs;
	malys->getAllTransformations(tfs);

	for (int i = 0; i < tfs.size(); i++){
		tfs[i].tx = (float)x[6 * i];
		tfs[i].ty = (float)x[6 * i + 1];
		tfs[i].tz = (float)x[6 * i + 2];
		tfs[i].ry = (float)x[6 * i + 3];
		tfs[i].rz = (float)x[6 * i + 4];
		tfs[i].rx = (float)x[6 * i + 5];
	}
	malys->setAllTransformations(tfs);

	std::vector<Vector3f> trans_points;
	//just for debug
	//for (int i = 0; i < points.size(); i++){
	//	std::vector<float> rot;
	//	std::vector<float> trans;
	//	malys->Transformation2RotTrans(tfs[i], rot, trans);
	//	Vector3f pt_tem = malys->TransformPoint(rot, trans, points[i]);

	//	trans_points.push_back(pt_tem);
	//}

	//std::vector<Vector3f> tnormals;
	//PointsIO::savePLYfile("tpoints.ply", trans_points, tnormals, Vector3u(0, 255, 0));
}

void MotionsData::updatePointsNormals()
{
	ITMRGBDCalib *calib = NULL;
	malys->getCalib(calib);

	for (int i = 0; i < points.size(); i++){
		if (visibles[i] != -1){
			Transformation tf = { x0[6 * i], x0[6 * i + 1], x0[6 * i + 2], x0[6 * i + 3], x0[6 * i + 4], x0[6 * i + 5] };
			Matrix4f mtf;
			malys->Transformation2Matrix4(tf, mtf);

			Vector4f vpt_tem(vpoints[visibles[i]].x, vpoints[visibles[i]].y, vpoints[visibles[i]].z, 1.0);
			Vector4f vpt = mtf * vpt_tem;
			Vector4f vn_tem(vnormals[visibles[i]].x, vnormals[visibles[i]].y, vnormals[visibles[i]].z, 1.0);
			Vector4f vn = mtf * vn_tem;

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

	//just for debug
	//std::vector<Vector3f> dpc;
	//for (int i = 0; i < depth_image_height; i++){
	//	for (int j = 0; j < depth_image_width; j++){
	//		Vector3f dpt;
	//		dpt.z = depth[j + i * depth_image_width];
	//		if (dpt.z != -1){
	//			dpt.x = dpt.z * ((float(j) - projParams_d.z) / projParams_d.x);
	//			dpt.y = dpt.z * ((float(i) - projParams_d.w) / projParams_d.y);
	//			dpc.push_back(dpt);
	//		}
	//	}
	//}
	//std::vector<Vector3f> dnormals;
	//PointsIO::savePLYfile("dpc.ply", dpc, dnormals, Vector3u(255, 255, 255));

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
		//Matrix4f mtf;
		//malys->Transformation2Matrix4(tf, mtf);
		std::vector<float> rot, trans;
		malys->Transformation2RotTrans(tf, rot, trans);

		// sum of data term
		Vector3f vi = points[i];
		Vector3f ni = normals[i];
		//Vector3f mvi = mtf * points[i];
		//Vector3f mni = mtf * normals[i];
		Vector3f mvi = malys->TransformPoint(rot, trans, points[i]);
		Vector3f mni = malys->TransformNormal(rot, normals[i]);
		Vector3f dvi = dpoints[visibles[i]];

		if (dvi.z == -1){
			continue;
		}		

		Vector3f delta_mvi_dvi(mvi.x - dvi.x, mvi.y - dvi.y, mvi.z - dvi.z);
		double dot1 = mni.x * delta_mvi_dvi.x + mni.y * delta_mvi_dvi.y + mni.z * delta_mvi_dvi.z;
		f += dot1 * dot1;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// gradient 
		// 2 * (ni.(vi - vi'))
		double dot2 = dot1;
		dot2 *= 2.0f;

		// tx, ty, tz gradient
		g[i * 6] += dot2 * mni.x;
		g[i * 6 + 1] += dot2 * mni.y;
		g[i * 6 + 2] += dot2 * mni.z;

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
		//Matrix4f ic;
		//malys->Transformation2Matrix4(tf_ic, ic);
		std::vector<float> iRot, iTrans;
		malys->Transformation2RotTrans(tf_ic, iRot, iTrans);

		//compute each sub term
		for (int j = 0; j < neighbors.size(); j++){
			unsigned int ind = neighbors[j];
			Transformation tf_jc = { x[6 * ind], x[6 * ind + 1], x[6 * ind + 2], x[6 * ind + 3], x[6 * ind + 4], x[6 * ind + 5] };
			//Matrix4f jc;
			//malys->Transformation2Matrix4(tf_jc, jc);
			std::vector<float> jRot, jTrans;
			malys->Transformation2RotTrans(tf_jc, jRot, jTrans);

			Vector3f vi = points[i];
			Vector3f vj = points[ind];
			//Vector3f mvij = ic * vj;
			//Vector3f mvjj = jc * vj;
			Vector3f mvij = malys->TransformPoint(iRot, iTrans, vj);
			Vector3f mvjj = malys->TransformPoint(jRot, jTrans, vj);

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
			g[i * 6 + 3] += (lambda / squared_distance) * 2.0f * (mvij.x - mvjj.x) * (chi_ * cai * vj.x + (shi_ * sbi - chi_ * sai * cbi) * vj.y + (chi_ * sai * sbi + shi_ * cbi) * vj.z);
			g[i * 6 + 3] += (lambda / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((-shi_ * cai) * vj.x + (shi_ * sai * cbi + chi_ * sbi) * vj.y + (-shi_ * sai * sbi + chi_ * cbi) * vj.z);

			// rz
			g[i * 6 + 4] += (lambda / squared_distance) * 2.0f * (mvij.x - mvjj.x) * ((chi * cai_) * vj.x + (-chi * sai_ * cbi) * vj.y + (chi * sai_ * sbi) * vj.z);
			g[i * 6 + 4] += (lambda / squared_distance) * 2.0f * (mvij.y - mvjj.y) * (sai_* vj.x + (cai_ * cbi) * vj.y + (-cai_ * sbi) * vj.z);
			g[i * 6 + 4] += (lambda / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((-shi * cai_)* vj.x + (shi * sai_ * cbi) * vj.y + (-shi * sai_ * sbi) * vj.z);

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
			g[ind * 6 + 3] += (-lambda / squared_distance) * 2.0f * (mvij.x - mvjj.x) * (chi_ * cai * vj.x + (shi_ * sbi - chi_ * sai * cbi) * vj.y + (chi_ * sai * sbi + shi_ * cbi) * vj.z);
			g[ind * 6 + 3] += (-lambda / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((-shi_ * cai) * vj.x + (shi_ * sai * cbi + chi_ * sbi) * vj.y + (-shi_ * sai * sbi + chi_ * cbi) * vj.z);

			// rz
			g[ind * 6 + 4] += (-lambda / squared_distance) * 2.0f * (mvij.x - mvjj.x) * ((chi * cai_) * vj.x + (-chi * sai_ * cbi) * vj.y + (chi * sai_ * sbi) * vj.z);
			g[ind * 6 + 4] += (-lambda / squared_distance) * 2.0f * (mvij.y - mvjj.y) * (sai_* vj.x + (cai_ * cbi) * vj.y + (-cai_ * sbi) * vj.z);
			g[ind * 6 + 4] += (-lambda / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((-shi * cai_)* vj.x + (shi * sai_ * cbi) * vj.y + (-shi * sai_ * sbi) * vj.z);

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
	printf("Iteration %d\n", k);
	printf("Iteration %d:  fx = %f,  xnorm = %f, gnorm = %f, step = %f\n", k, fx, xnorm, gnorm, step);
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// main optimization function
void ITMMotionAnalysis::optimizeEnergyFunction(ITMFloatImage *newDepthImage)
{
	MotionsData data(this, newDepthImage);

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

	std::vector<Transformation> ctfs;
	getAllTransformations(ctfs);
	float f = 0;
	for (unsigned int i = 0; i < data.points.size(); i++) {
		if (data.visibles[i] == -1)
			continue;

		std::vector<float> rot, trans;
		Transformation2RotTrans(ctfs[i], rot, trans);

		// sum of data term
		Vector3f vi = data.points[i];
		Vector3f ni = data.normals[i];
		Vector3f mvi = TransformPoint(rot, trans, data.points[i]);
		Vector3f mni = TransformNormal(rot, data.normals[i]);
		Vector3f dvi = data.dpoints[data.visibles[i]];

		if (dvi.z == -1){
			continue;
		}

		Vector3f delta_mvi_dvi(mvi.x - dvi.x, mvi.y - dvi.y, mvi.z - dvi.z);
		double dot1 = mni.x * delta_mvi_dvi.x + mni.y * delta_mvi_dvi.y + mni.z * delta_mvi_dvi.z;
		f += dot1 * dot1;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// objective function
double motions_function(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
	MotionsData* d = (MotionsData*)data;
	const std::vector<Vector3f>& points = d->points;
	const std::vector<Vector3f>& normals = d->normals;
	const std::vector<Vector3f>& dpoints = d->dpoints;
	const std::vector<double>& x0 = d->x0;
	const std::vector<int>& visibles = d->visibles;
	const std::vector<bool>& livelist = d->livelist;
	const std::vector<std::vector<unsigned int>>& neighborhood = d->neighborhood;
	ITMMotionAnalysis* malys = d->malys;
	//float lambda = d->lambda;
	float alfa = d->alfa;
	float beta = d->beta;
	//////////////////////////////////////////////////////////////////////////
	// initialize
	double f = 0;
	for (int i = 0; i < (int)x.size(); ++i) {
		grad[i] = 0;
	}

	// data term
	for (int i = 0; i < (int)x.size() / 6; i++) {
		if (visibles[i] == -1)
			continue;

		// warp transformation of i
		Transformation tf = { x[6 * i], x[6 * i + 1], x[6 * i + 2], x[6 * i + 3], x[6 * i + 4], x[6 * i + 5] };
		std::vector<float> rot, trans;
		malys->Transformation2RotTrans(tf, rot, trans);

		// sum of data term
		Vector3f vi = points[i];
		Vector3f ni = normals[i];
		Vector3f mvi = malys->TransformPoint(rot, trans, points[i]);
		Vector3f mni = malys->TransformNormal(rot, normals[i]);
		Vector3f dvi = dpoints[visibles[i]];

		if (dvi.z == -1){
			continue;
		}

		Vector3f delta_mvi_dvi(mvi.x - dvi.x, mvi.y - dvi.y, mvi.z - dvi.z);
		double dot1 = mni.x * delta_mvi_dvi.x + mni.y * delta_mvi_dvi.y + mni.z * delta_mvi_dvi.z;
		f += dot1 * dot1;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// gradient 
		// 2 * (ni.(vi - vi'))
		double dot2 = dot1;
		dot2 *= 2.0f;

		// tx, ty, tz gradient
		grad[i * 6] += dot2 * mni.x;
		grad[i * 6 + 1] += dot2 * mni.y;
		grad[i * 6 + 2] += dot2 * mni.z;

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

		// ry gradient
		// (mtf * ni) . (mtf * vi) - (mtf * ni) . dvi gradient
		grad[i * 6 + 3] += (mvi.x - dvi.x) * (chi_ * cai * ni.x + (shi_ * sbi - chi_ * sai * cbi) * ni.y + (chi_ * sai * sbi + shi_ * cbi) * ni.z);
		grad[i * 6 + 3] += mni.x * (chi_ * cai * vi.x + (shi_ * sbi - chi_ * sai * cbi) * vi.y + (chi_ * sai * sbi + shi_ * cbi) * vi.z);
		grad[i * 6 + 3] += (mvi.z - dvi.z) * (-shi_ * cai * ni.x + (shi_ * sai * cbi + chi_ * sbi) * ni.y + (-shi_ * sai * sbi + chi_ * cbi) * ni.z);
		grad[i * 6 + 3] += mni.z * (-shi_ * cai * vi.x + (shi_ * sai * cbi + chi_ * sbi) * vi.y + (-shi_ * sai * sbi + chi_ * cbi) * vi.z);
		// 2 * (ni.(vi - vi')) * ((mtf * ni) . (mtf * vi) - (mtf * ni) . dvi)'ry
		grad[i * 6 + 3] *= dot2;

		// rz gradient
		// (mtf * ni) . (mtf * vi) - (mtf * ni) . dvi gradient
		grad[i * 6 + 4] += (mvi.x - dvi.x) * (chi * cai_ * ni.x + (-chi * sai_ * cbi) * ni.y + (chi * sai_ * sbi) * ni.z);
		grad[i * 6 + 4] += mni.x * (chi * cai_ * vi.x + (-chi * sai_ * cbi) * vi.y + (chi * sai_ * sbi) * vi.z);
		grad[i * 6 + 4] += (mvi.y - dvi.y) * (sai_ * ni.x + cai_ * cbi * ni.y + (-cai_ * sbi) * ni.z);
		grad[i * 6 + 4] += mni.y * (sai_ * vi.x + cai_ * cbi * vi.y + (-cai_ * sbi) * vi.z);
		grad[i * 6 + 4] += (mvi.z - dvi.z) * ((-shi * cai_)* ni.x + shi * sai_ * cbi * ni.y + (-shi * sai_ * sbi) * ni.z);
		grad[i * 6 + 4] += mni.z * ((-shi * cai_)* vi.x + shi * sai_ * cbi * vi.y + (-shi * sai_ * sbi) * vi.z);
		// 2 * (ni.(vi - vi')) * ((mtf * ni) . (mtf * vi) - (mtf * ni) . dvi)'rz
		grad[i * 6 + 4] *= dot2;

		// rx gradient
		// (mtf * ni) . (mtf * vi) - (mtf * ni) . dvi gradient
		grad[i * 6 + 5] += (mvi.x - dvi.x) * ((shi * sbi_ - chi * sai * cbi_)* ni.y + (chi * sai * sbi_ + shi * cbi_) * ni.z);
		grad[i * 6 + 5] += mni.x * ((shi * sbi_ - chi * sai * cbi_)* vi.y + (chi * sai * sbi_ + shi * cbi_) * vi.z);
		grad[i * 6 + 5] += (mvi.y - dvi.y) * (cai * cbi_ * ni.y + (-cai * sbi_) * ni.z);
		grad[i * 6 + 5] += mni.y * (cai * cbi_ * vi.y + (-cai * sbi_) * vi.z);
		grad[i * 6 + 5] += (mvi.z - dvi.z) * ((shi * sai * cbi_ + chi * cbi_) * ni.y + (-shi * sai * sbi_ + chi * cbi_) * ni.z);
		grad[i * 6 + 5] += mni.z * ((shi * sai * cbi_ + chi * cbi_) * vi.y + (-shi * sai * sbi_ + chi * cbi_) * vi.z);
		// 2 * (ni.(vi - vi')) * ((mtf * ni) . (mtf * vi) - (mtf * ni) . dvi)'rx
		grad[i * 6 + 5] *= dot2;
	}

	// reg term
	for (int i = 0; i < (int)x.size() / 6; i++){
		const std::vector<unsigned int>& neighbors = neighborhood[i];
		Transformation tf_ic = { x[6 * i], x[6 * i + 1], x[6 * i + 2], x[6 * i + 3], x[6 * i + 4], x[6 * i + 5] };
		std::vector<float> iRot, iTrans;
		malys->Transformation2RotTrans(tf_ic, iRot, iTrans);

		float factor = 0.0f;

		if (livelist[i]){
			factor = alfa;
		}
		else{
			factor = beta;
		}

		//compute each sub term
		for (int j = 0; j < neighbors.size(); j++){
			unsigned int ind = neighbors[j];
			if (ind == i){
				continue;
			}

			Transformation tf_jc = { x[6 * ind], x[6 * ind + 1], x[6 * ind + 2], x[6 * ind + 3], x[6 * ind + 4], x[6 * ind + 5] };
			std::vector<float> jRot, jTrans;
			malys->Transformation2RotTrans(tf_jc, jRot, jTrans);

			Vector3f vi = points[i];
			Vector3f vj = points[ind];
			Vector3f mvij = malys->TransformPoint(iRot, iTrans, vj);
			Vector3f mvjj = malys->TransformPoint(jRot, jTrans, vj);

			// sum of reg term
			Vector3f vij = vi - vj;
			float squared_distance = vij.x * vij.x + vij.y * vij.y + vij.z * vij.z;
			Vector3f delta = mvij - mvjj;
			double result_tem = (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) / squared_distance;
			f += factor * result_tem;

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
			grad[i * 6] += factor * 2.0f * (mvij.x - mvjj.x) * 1.0f / squared_distance;
			grad[i * 6 + 1] += factor * 2.0f * (mvij.y - mvjj.y) * 1.0f / squared_distance;
			grad[i * 6 + 2] += factor * 2.0f * (mvij.z - mvjj.z) * 1.0f / squared_distance;

			// i, ry, rz, rx gradient
			// ry
			grad[i * 6 + 3] += (factor / squared_distance) * 2.0f * (mvij.x - mvjj.x) * (chi_ * cai * vj.x + (shi_ * sbi - chi_ * sai * cbi) * vj.y + (chi_ * sai * sbi + shi_ * cbi) * vj.z);
			grad[i * 6 + 3] += (factor / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((-shi_ * cai) * vj.x + (shi_ * sai * cbi + chi_ * sbi) * vj.y + (-shi_ * sai * sbi + chi_ * cbi) * vj.z);

			// rz
			grad[i * 6 + 4] += (factor / squared_distance) * 2.0f * (mvij.x - mvjj.x) * ((chi * cai_) * vj.x + (-chi * sai_ * cbi) * vj.y + (chi * sai_ * sbi) * vj.z);
			grad[i * 6 + 4] += (factor / squared_distance) * 2.0f * (mvij.y - mvjj.y) * (sai_* vj.x + (cai_ * cbi) * vj.y + (-cai_ * sbi) * vj.z);
			grad[i * 6 + 4] += (factor / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((-shi * cai_)* vj.x + (shi * sai_ * cbi) * vj.y + (-shi * sai_ * sbi) * vj.z);

			// rx
			grad[i * 6 + 5] += (factor / squared_distance) * 2.0f * (mvij.x - mvjj.x) * ((shi * sbi_ - chi * sai * cbi_) * vj.y + (chi * sai * sbi_ + shi * cbi_) * vj.z);
			grad[i * 6 + 5] += (factor / squared_distance) * 2.0f * (mvij.y - mvjj.y) * ((cai * cbi_)* vj.y + (-cai * sbi_) * vj.z);
			grad[i * 6 + 5] += (factor / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((shi * sai * cbi_ + chi * sbi_) * vj.y + (-shi * sai * sbi_ + chi * cbi_) * vj.z);

			// j, tx, ty, tz
			grad[ind * 6] += factor * 2.0f * (mvij.x - mvjj.x) * (-1.0f) / squared_distance;
			grad[ind * 6 + 1] += factor * 2.0f * (mvij.y - mvjj.y) * (-1.0f) / squared_distance;
			grad[ind * 6 + 2] += factor * 2.0f * (mvij.z - mvjj.z) * (-1.0f) / squared_distance;

			// j, ry, rz, rx gradient
			// ry
			grad[ind * 6 + 3] += (-factor / squared_distance) * 2.0f * (mvij.x - mvjj.x) * (chi_ * cai * vj.x + (shi_ * sbi - chi_ * sai * cbi) * vj.y + (chi_ * sai * sbi + shi_ * cbi) * vj.z);
			grad[ind * 6 + 3] += (-factor / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((-shi_ * cai) * vj.x + (shi_ * sai * cbi + chi_ * sbi) * vj.y + (-shi_ * sai * sbi + chi_ * cbi) * vj.z);

			// rz
			grad[ind * 6 + 4] += (-factor / squared_distance) * 2.0f * (mvij.x - mvjj.x) * ((chi * cai_) * vj.x + (-chi * sai_ * cbi) * vj.y + (chi * sai_ * sbi) * vj.z);
			grad[ind * 6 + 4] += (-factor / squared_distance) * 2.0f * (mvij.y - mvjj.y) * (sai_* vj.x + (cai_ * cbi) * vj.y + (-cai_ * sbi) * vj.z);
			grad[ind * 6 + 4] += (-factor / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((-shi * cai_)* vj.x + (shi * sai_ * cbi) * vj.y + (-shi * sai_ * sbi) * vj.z);

			// rx
			grad[ind * 6 + 5] += (-factor / squared_distance) * 2.0f * (mvij.x - mvjj.x) * ((shi * sbi_ - chi * sai * cbi_) * vj.y + (chi * sai * sbi_ + shi * cbi_) * vj.z);
			grad[ind * 6 + 5] += (-factor / squared_distance) * 2.0f * (mvij.y - mvjj.y) * ((cai * cbi_)* vj.y + (-cai * sbi_) * vj.z);
			grad[ind * 6 + 5] += (-factor / squared_distance) * 2.0f * (mvij.z - mvjj.z) * ((shi * sai * cbi_ + chi * sbi_) * vj.y + (-shi * sai * sbi_ + chi * cbi_) * vj.z);
		}
	}

	return f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// main optimization function
void ITMMotionAnalysis::optimizeEnergyFunctionNlopt(ITMFloatImage *newDepthImage)
{
	MotionsData data(this, newDepthImage);

	int n = data.x0.size();
	std::vector<double> x(n);
	for (int i = 0; i < n; ++i) {
		x[i] = data.x0[i];
	}

	nlopt::opt opt(nlopt::LD_MMA, n);
	opt.set_min_objective(motions_function, &data);
	opt.set_xtol_rel(1e-5);

	double minf;
	nlopt::result result = opt.optimize(x, minf);

	std::cout << "minf = " << minf << std::endl;
	data.updateAllWarpInfo(x);
}