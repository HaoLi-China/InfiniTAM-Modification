// Hao added it
#include <math.h>

#include "ITMMotionAnalysis.h"
#include "../../Utils/KDtree/kdtree_search_eth.h"
#include "../../Utils/PointsIO/PointsIO.h"

using namespace ITMLib::Engine;

ITMMotionAnalysis::ITMMotionAnalysis(const ITMRGBDCalib *calib, bool useControlPoints){
	this->calib = const_cast<ITMRGBDCalib*>(calib);
	this->useControlPoints = useControlPoints;
	this->changeDpWhenIteration = true;

	findDepthPointsPolicy = 0; // 0:Just do projection        1:Choose dpoint within a rect region.
}

ITMMotionAnalysis::~ITMMotionAnalysis(){
	
}

void ITMMotionAnalysis::initialize(std::vector<Vector3f> &cpoints, std::vector<Vector3f> &cnormals, std::vector<bool> &visiblelist, std::vector<std::vector<unsigned int>> visibleNeighbors){
	this->cpoints = cpoints;
	this->cnormals = cnormals;
	this->visiblelist = visiblelist;
	this->visibleNeighbors = visibleNeighbors;

	ctfs.clear();
	for (int i = 0; i < cpoints.size(); i++){
		Transformation tf = { 0, 0, 0, 0, 0, 0 };
		ctfs.push_back(tf);
	}
}

void ITMMotionAnalysis::getCalib(ITMRGBDCalib *&calib){
	calib = this->calib;
}

void ITMMotionAnalysis::getAllPoints(std::vector<Vector3f> &cpoints){
	cpoints = this->cpoints;
}

void ITMMotionAnalysis::getAllNormals(std::vector<Vector3f> &cnormals){
	cnormals = this->cnormals;
}

void ITMMotionAnalysis::getAllTransformations(std::vector<Transformation> &ctfs){
	ctfs = this->ctfs;
}

void ITMMotionAnalysis::setAllTransformations(const std::vector<Transformation> &ctfs){
	this->ctfs = ctfs;
}

void ITMMotionAnalysis::getAllVisibleList(std::vector<bool> &visiblelist){
	visiblelist = this->visiblelist;
}

void ITMMotionAnalysis::getVisibleNeighbors(std::vector<std::vector<unsigned int>> &visibleNeighbors){
	visibleNeighbors = this->visibleNeighbors;
}

//infer transformation of new points
void ITMMotionAnalysis::inferTransformations(const std::vector<Vector3f> &cpoints, const std::vector<Transformation> &ctfs, const std::vector<Vector3f> &npoints, std::vector<Transformation> &ntfs){
	ntfs.clear();
	
	Vector3f *pointSet = (Vector3f*)malloc((cpoints.size())*sizeof(Vector3f));
	for (int i = 0; i < cpoints.size(); i++){
		pointSet[i].x = cpoints[i].x;
		pointSet[i].y = cpoints[i].y;
		pointSet[i].z = cpoints[i].z;
	}

	KdTreeSearch_ETH kd_eth;
	kd_eth.add_vertex_set(pointSet, cpoints.size());
	kd_eth.end();

	for (int i = 0; i < npoints.size(); i++){
		Vector3f p = npoints[i];

		//get neighbor points within a range of radius
		std::vector<unsigned int> neighbors;
		//kd_eth.find_points_in_radius(p, INFLUENCE_RADIUS*INFLUENCE_RADIUS, neighbors); 
		kd_eth.find_closest_K_points(p, 1, neighbors);

		if (neighbors.size() == 0){
			std::cout << "p.x:" << p.x << std::endl;
			std::cout << "p.y:" << p.y << std::endl;
			std::cout << "p.z:" << p.z << std::endl;
		}

		std::vector<double> weights;
		double weight_sum = 0;
		Transformation trans_res = { 0, 0, 0, 0, 0, 0 };

		for (int k = 0; k < neighbors.size(); k++){
			unsigned int index = neighbors[k];
			double squared_dis = (cpoints[index].x - p.x)*(cpoints[index].x - p.x) + (cpoints[index].y - p.y)*(cpoints[index].y - p.y) + (cpoints[index].z - p.z)*(cpoints[index].z - p.z);

			double weight_tem = exp(-squared_dis / (2.0f*INFLUENCE_RADIUS*INFLUENCE_RADIUS));
			weights.push_back(weight_tem);
			weight_sum += weight_tem;
		}

		//normalize the weight
		for (int k = 0; k < weights.size(); k++){
			weights[k] /= weight_sum;
		}

		//compute the new transformation
		for (int k = 0; k < neighbors.size(); k++){
			unsigned int index = neighbors[k];
			trans_res.tx += weights[k] * ctfs[index].tx;
			trans_res.ty += weights[k] * ctfs[index].ty;
			trans_res.tz += weights[k] * ctfs[index].tz;
			trans_res.ry += weights[k] * ctfs[index].ry;
			trans_res.rz += weights[k] * ctfs[index].rz;
			trans_res.rx += weights[k] * ctfs[index].rx;
		}

		ntfs.push_back(trans_res);
	}

	kd_eth.begin();
	free(pointSet);
	pointSet = NULL;
}

//transform all points
void ITMMotionAnalysis::transformAllPoints(std::vector<Vector3f> &cpoints, std::vector<Vector3f> &cnormals){
	if (useControlPoints){
		//transform control points
		for (int i = 0; i < cpoints.size(); i++){
			std::vector<float> rot, trans;
			Transformation2RotTrans(ctfs[i], rot, trans);
			cpoints[i] = TransformPoint(rot, trans, cpoints[i]);
			cnormals[i] = TransformNormal(rot, cnormals[i]);
		}
	}
}

void ITMMotionAnalysis::Transformation2Matrix4(const Transformation &tf, Matrix4f &mtf){
	mtf.setIdentity();

	// Assuming the angles are in radians.
	double ch = cos(tf.ry);
	double sh = sin(tf.ry);
	double ca = cos(tf.rz);
	double sa = sin(tf.rz);
	double cb = cos(tf.rx);
	double sb = sin(tf.rx);

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

void ITMMotionAnalysis::Transformation2RotTrans(const Transformation &tf, std::vector<float>& rot, std::vector<float>& trans)
{
	rot.clear();
	trans.clear();

	float m00, m01, m02, m10, m11, m12, m20, m21, m22, t0, t1, t2;

	// Assuming the angles are in radians.
	double ch = cos(tf.ry);
	double sh = sin(tf.ry);
	double ca = cos(tf.rz);
	double sa = sin(tf.rz);
	double cb = cos(tf.rx);
	double sb = sin(tf.rx);

	m00 = ch * ca;
	m01 = sh*sb - ch*sa*cb;
	m02 = ch*sa*sb + sh*cb;
	m10 = sa;
	m11 = ca*cb;
	m12 = -ca*sb;
	m20 = -sh*ca;
	m21 = sh*sa*cb + ch*sb;
	m22 = -sh*sa*sb + ch*cb;
	t0 = tf.tx;
	t1 = tf.ty;
	t2 = tf.tz;

	rot.push_back(m00);
	rot.push_back(m01);
	rot.push_back(m02);
	rot.push_back(m10);
	rot.push_back(m11);
	rot.push_back(m12);
	rot.push_back(m20);
	rot.push_back(m21);
	rot.push_back(m22);

	trans.push_back(t0);
	trans.push_back(t1);
	trans.push_back(t2);
}

Vector3f ITMMotionAnalysis::TransformPoint(const std::vector<float>& rot, const std::vector<float>& trans, const Vector3f& point)
{
	float x = rot[0] * point.x + rot[1] * point.y + rot[2] * point.z + trans[0];
	float y = rot[3] * point.x + rot[4] * point.y + rot[5] * point.z + trans[1];
	float z = rot[6] * point.x + rot[7] * point.y + rot[8] * point.z + trans[2];

	Vector3f tPoint(x, y, z);
	return tPoint;
}

Vector3f ITMMotionAnalysis::TransformNormal(const std::vector<float>& rot, const Vector3f& normal)
{
	float nx = rot[0] * normal.x + rot[1] * normal.y + rot[2] * normal.z;
	float ny = rot[3] * normal.x + rot[4] * normal.y + rot[5] * normal.z;
	float nz = rot[6] * normal.x + rot[7] * normal.y + rot[8] * normal.z;

	Vector3f tNormal(nx, ny, nz);
	return tNormal;
}

void ITMMotionAnalysis::RotTrans2Transformation(const std::vector<float>& rot, const std::vector<float>& trans, Transformation &tf)
{
	float m00 = rot[0];
	float m01 = rot[1];
	float m02 = rot[2];
	float m10 = rot[3];
	float m11 = rot[4];
	float m12 = rot[5];
	float m20 = rot[6];
	float m21 = rot[7];
	float m22 = rot[8];
	float t0 = trans[0];
	float t1 = trans[1];
	float t2 = trans[2];

	tf.tx = t0;
	tf.ty = t1;
	tf.tz = t2;

	if (m10 > 0.998) { // singularity at north pole
		tf.ry = atan2(m02, m22);
		tf.rz = PI / 2;
		tf.rx = 0;
		return;
	}
	if (m10 < -0.998) { // singularity at south pole
		tf.ry = atan2(m02, m22);
		tf.rz = PI / 2;
		tf.rx = 0;
		return;
	}
	tf.ry = atan2(-m20, m00);
	tf.rx = atan2(-m12, m11);
	tf.rz = asin(m10);
}

void ITMMotionAnalysis::Matrix42Transformation(const Matrix4f &mtf, Transformation &tf){
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

void ITMMotionAnalysis::getNeighboorsOfEachNode(const std::vector<Vector3f> &cpoints, const float radius, std::vector<std::vector<unsigned int>> &neighbor){
	Vector3f *pointSet = (Vector3f*)malloc((cpoints.size())*sizeof(Vector3f));
	for (int i = 0; i < cpoints.size(); i++){
		pointSet[i].x = cpoints[i].x;
		pointSet[i].y = cpoints[i].y;
		pointSet[i].z = cpoints[i].z;
	}

	KdTreeSearch_ETH kd_eth;
	kd_eth.add_vertex_set(pointSet, cpoints.size());
	kd_eth.end();

	for (int i = 0; i < cpoints.size(); i++){
		Vector3f p = cpoints[i];

		//get neighbor points within a range of radius
		std::vector<unsigned int> neighbors;
		kd_eth.find_points_in_radius(p, radius, neighbors);
		neighbor.push_back(neighbors);
	}

	kd_eth.begin();
	free(pointSet);
	pointSet = NULL;
}

bool ITMMotionAnalysis::invTransformation(const std::vector<float>& source_rot, const std::vector<float>& source_trans, std::vector<float>& target_rot, std::vector<float>& target_trans){
	target_rot.resize(source_rot.size());
	target_trans.resize(source_trans.size());
	
	float determinant = (source_rot[4] * source_rot[8] - source_rot[5] * source_rot[7])*source_rot[0] + (source_rot[5] * source_rot[6] - source_rot[3] * source_rot[8])*source_rot[1] + (source_rot[3] * source_rot[7] - source_rot[4] * source_rot[6])*source_rot[2];

	if (determinant == 0) {
		target_rot[0] = 0;
		target_rot[1] = 0;
		target_rot[2] = 0;
		target_rot[3] = 0;
		target_rot[4] = 0;
		target_rot[5] = 0;
		target_rot[6] = 0;
		target_rot[7] = 0;
		target_rot[8] = 0;

		target_trans[0] = 0;
		target_trans[1] = 0;
		target_trans[2] = 0;
		return false;
	}

	target_rot[0] = (source_rot[4] * source_rot[8] - source_rot[5] * source_rot[7]) / determinant;
	target_rot[1] = (source_rot[2] * source_rot[7] - source_rot[1] * source_rot[8]) / determinant;
	target_rot[2] = (source_rot[1] * source_rot[5] - source_rot[2] * source_rot[4]) / determinant;
	target_rot[3] = (source_rot[5] * source_rot[6] - source_rot[3] * source_rot[8]) / determinant;
	target_rot[4] = (source_rot[0] * source_rot[8] - source_rot[2] * source_rot[6]) / determinant;
	target_rot[5] = (source_rot[2] * source_rot[3] - source_rot[0] * source_rot[5]) / determinant;
	target_rot[6] = (source_rot[3] * source_rot[7] - source_rot[4] * source_rot[6]) / determinant;
	target_rot[7] = (source_rot[1] * source_rot[6] - source_rot[0] * source_rot[7]) / determinant;
	target_rot[8] = (source_rot[0] * source_rot[4] - source_rot[1] * source_rot[3]) / determinant;

	target_trans[0] = -source_trans[0];
	target_trans[1] = -source_trans[1];
	target_trans[2] = -source_trans[2];

	return true;
	/*
	target_rot.resize(source_rot.size());
	target_trans.resize(source_trans.size());
	
	float tmp[12], src[16];
	float det;

	src[0] = source_rot[0];
	src[1] = source_rot[1];
	src[2] = source_rot[2];
	src[3] = source_trans[0];
	src[4] = source_rot[3];
	src[5] = source_rot[4];
	src[6] = source_rot[5];
	src[7] = source_trans[1];
	src[8] = source_rot[6];
	src[9] = source_rot[7];
	src[10] = source_rot[8];
	src[11] = source_trans[2];
	src[12] = 0;
	src[13] = 0;
	src[14] = 0;
	src[15] = 1;

	tmp[0] = src[10] * src[15];
	tmp[1] = src[11] * src[14];
	tmp[2] = src[9] * src[15];
	tmp[3] = src[11] * src[13];
	tmp[4] = src[9] * src[14];
	tmp[5] = src[10] * src[13];
	tmp[6] = src[8] * src[15];
	tmp[7] = src[11] * src[12];
	tmp[8] = src[8] * src[14];
	tmp[9] = src[10] * src[12];
	tmp[10] = src[8] * src[13];
	tmp[11] = src[9] * src[12];

	target_rot[0] = ((tmp[0] * src[5] + tmp[3] * src[6] + tmp[4] * src[7]) - (tmp[1] * src[5] + tmp[2] * src[6] + tmp[5] * src[7]));
	target_rot[3] = ((tmp[1] * src[4] + tmp[6] * src[6] + tmp[9] * src[7]) - (tmp[0] * src[4] + tmp[7] * src[6] + tmp[8] * src[7]));
	target_rot[6] = ((tmp[2] * src[4] + tmp[7] * src[5] + tmp[10] * src[7]) - (tmp[3] * src[4] + tmp[6] * src[5] + tmp[11] * src[7]));

	det = src[0] * src[0] + src[1] * src[4] + src[2] * src[8] + src[3] * src[12];
	if (det == 0.0f)
		return false;

	target_rot[0] /= det;
	target_rot[3] /= det;
	target_rot[6] /= det;

	target_rot[1] = ((tmp[1] * src[1] + tmp[2] * src[2] + tmp[5] * src[3]) - (tmp[0] * src[1] + tmp[3] * src[2] + tmp[4] * src[3])) / det;
	target_rot[4] = ((tmp[0] * src[0] + tmp[7] * src[2] + tmp[8] * src[3]) - (tmp[1] * src[0] + tmp[6] * src[2] + tmp[9] * src[3])) / det;
	target_rot[7] = ((tmp[3] * src[0] + tmp[6] * src[1] + tmp[11] * src[3]) - (tmp[2] * src[0] + tmp[7] * src[1] + tmp[10] * src[3])) / det;

	tmp[0] = src[2] * src[7];
	tmp[1] = src[3] * src[6];
	tmp[2] = src[1] * src[7];
	tmp[3] = src[3] * src[5];
	tmp[4] = src[1] * src[6];
	tmp[5] = src[2] * src[5];
	tmp[6] = src[0] * src[7];
	tmp[7] = src[3] * src[4];
	tmp[8] = src[0] * src[6];
	tmp[9] = src[2] * src[4];
	tmp[10] = src[0] * src[5];
	tmp[11] = src[1] * src[4];

	target_rot[2] = ((tmp[0] * src[13] + tmp[3] * src[14] + tmp[4] * src[15]) - (tmp[1] * src[13] + tmp[2] * src[14] + tmp[5] * src[15])) / det;
	target_rot[5] = ((tmp[1] * src[12] + tmp[6] * src[14] + tmp[9] * src[15]) - (tmp[0] * src[12] + tmp[7] * src[14] + tmp[8] * src[15])) / det;
	target_rot[8] = ((tmp[2] * src[12] + tmp[7] * src[13] + tmp[10] * src[15]) - (tmp[3] * src[12] + tmp[6] * src[13] + tmp[11] * src[15])) / det;


	target_trans[0] = ((tmp[2] * src[10] + tmp[5] * src[11] + tmp[1] * src[9]) - (tmp[4] * src[11] + tmp[0] * src[9] + tmp[3] * src[10])) / det;
	target_trans[1] = ((tmp[8] * src[11] + tmp[0] * src[8] + tmp[7] * src[10]) - (tmp[6] * src[10] + tmp[9] * src[11] + tmp[1] * src[8])) / det;
	target_trans[2] = ((tmp[6] * src[9] + tmp[11] * src[11] + tmp[3] * src[8]) - (tmp[10] * src[11] + tmp[2] * src[8] + tmp[7] * src[9])) / det;

	return true;*/
}


//upadate transformation
void ITMMotionAnalysis::updateTransformation(Transformation &step, Transformation &oldTransformation, Transformation &newTransformation){
	std::vector<float> global_rot, new_rot;
	std::vector<float> global_trans, new_trans;
	Transformation2RotTrans(oldTransformation, global_rot, global_trans);
	Transformation2RotTrans(step, new_rot, new_trans);

	float r00 = new_rot[0] * global_rot[0] + new_rot[1] * global_rot[3] + new_rot[2] * global_rot[6];
	float r01 = new_rot[0] * global_rot[1] + new_rot[1] * global_rot[4] + new_rot[2] * global_rot[7];
	float r02 = new_rot[0] * global_rot[2] + new_rot[1] * global_rot[5] + new_rot[2] * global_rot[8];

	float r10 = new_rot[3] * global_rot[0] + new_rot[4] * global_rot[3] + new_rot[5] * global_rot[6];
	float r11 = new_rot[3] * global_rot[1] + new_rot[4] * global_rot[4] + new_rot[5] * global_rot[7];
	float r12 = new_rot[3] * global_rot[2] + new_rot[4] * global_rot[5] + new_rot[5] * global_rot[8];

	float r20 = new_rot[6] * global_rot[0] + new_rot[7] * global_rot[3] + new_rot[8] * global_rot[6];
	float r21 = new_rot[6] * global_rot[1] + new_rot[7] * global_rot[4] + new_rot[8] * global_rot[7];
	float r22 = new_rot[6] * global_rot[2] + new_rot[7] * global_rot[5] + new_rot[8] * global_rot[8];

	float t0 = new_rot[0] * global_trans[0] + new_rot[1] * global_trans[1] + new_rot[2] * global_trans[2];
	float t1 = new_rot[3] * global_trans[0] + new_rot[4] * global_trans[1] + new_rot[5] * global_trans[2];
	float t2 = new_rot[6] * global_trans[0] + new_rot[7] * global_trans[1] + new_rot[8] * global_trans[2];

	t0 += new_trans[0];
	t1 += new_trans[1];
	t2 += new_trans[2];

	global_rot[0] = r00;
	global_rot[1] = r01;
	global_rot[2] = r02;

	global_rot[3] = r10;
	global_rot[4] = r11;
	global_rot[5] = r12;

	global_rot[6] = r20;
	global_rot[7] = r21;
	global_rot[8] = r22;

	global_trans[0] = t0;
	global_trans[1] = t1;
	global_trans[2] = t2;

	RotTrans2Transformation(global_rot, global_trans, newTransformation);
}

Vector3f ITMMotionAnalysis::interpolateBilinear(const std::vector<Vector3f> &source, const Vector2f & position, const Vector2i &imgSize){
	Vector3f a, b, c, d;
	Vector3f result;
	Vector2s p;
	Vector2f delta;

	p.x = (short)floor(position.x);
	p.y = (short)floor(position.y);
	delta.x = position.x - (float)p.x;
	delta.y = position.y - (float)p.y;

	a = source[p.x + p.y * imgSize.x];
	b = source[(p.x + 1) + p.y * imgSize.x];
	c = source[p.x + (p.y + 1) * imgSize.x];
	d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	if (a.z < 0 || b.z < 0 || c.z < 0 || d.z < 0)
	{
		result.x = 0;
		result.y = 0;
		result.z = -1.0f;
		return result;
	}

	result.x = ((float)a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.x * delta.x * (1.0f - delta.y) +
		(float)c.x * (1.0f - delta.x) * delta.y + (float)d.x * delta.x * delta.y);
	result.y = ((float)a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.y * delta.x * (1.0f - delta.y) +
		(float)c.y * (1.0f - delta.x) * delta.y + (float)d.y * delta.x * delta.y);
	result.z = ((float)a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.z * delta.x * (1.0f - delta.y) +
		(float)c.z * (1.0f - delta.x) * delta.y + (float)d.z * delta.x * delta.y);

	return result;
}
