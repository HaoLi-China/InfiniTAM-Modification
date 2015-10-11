// Hao added it
#include <math.h>

#include "ITMMotionAnalysis.h"
#include "../../Utils/KDtree/kdtree_search_eth.h"
#include "../../Utils/PointsIO/PointsIO.h"

using namespace ITMLib::Engine;

ITMMotionAnalysis::ITMMotionAnalysis(const ITMRGBDCalib *calib, bool useControlPoints){
	this->calib = const_cast<ITMRGBDCalib*>(calib);
	this->useControlPoints = useControlPoints;
	this->changeDpWhenIteration = false;

	findDepthPointsPolicy = 1; // 0:Just do projection       1:Choose dpoint within a rect region.
	regTermPolicy = 5; // 0:Modified DynamicFusion    1:Sig2014    2:DynamicFusion   3:as-rigid  4.modified as-rigid   5.new modified as-rigid 
	dataTermPolicy = 1; //0:pointsWithNormals 1:pointWithNormals and only points
}

ITMMotionAnalysis::~ITMMotionAnalysis(){
	
}

void ITMMotionAnalysis::initialize(std::vector<Vector3f> &cpoints, std::vector<Vector3f> &cnormals, std::vector<bool> &visiblelist){
	this->cpoints = cpoints;
	this->cnormals = cnormals;
	this->visiblelist = visiblelist;

	ctfs.clear();
	for (int i = 0; i < cpoints.size(); i++){
		Transformation tf = {0, 0, 0, 0, 0, 0};
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

void ITMMotionAnalysis::getAllOperationPointsTransformation(const std::vector<Vector3f> &points, std::vector<Vector3f> &cpoints, std::vector<Vector3f> &cnormals, std::vector<Transformation> &tfs){
	    
	    Vector3f *pointSet = (Vector3f*)malloc((cpoints.size())*sizeof(Vector3f));
		for (int i = 0; i < cpoints.size(); i++){
			pointSet[i].x = cpoints[i].x;
			pointSet[i].y = cpoints[i].y;
			pointSet[i].z = cpoints[i].z;
		}

		KdTreeSearch_ETH kd_eth;
		kd_eth.add_vertex_set(pointSet, cpoints.size());
		kd_eth.end();

		for (int i = 0; i < points.size(); i++){
			Vector3f p = points[i];

			//get neighbor points within a range of radius
			std::vector<unsigned int> neighbors;
			//kd_eth.find_points_in_radius(p, INFLUENCE_RADIUS*INFLUENCE_RADIUS, neighbors); 
			kd_eth.find_closest_K_points(p, 1, neighbors);

			if (neighbors.size() == 0){
				std::cout << "p.x:" << p.x << std::endl;
				std::cout << "p.y:" << p.y << std::endl;
				std::cout << "p.z:" << p.z << std::endl;
			}


			/*if (neighbors.size()==0){
				printf("neighbors.size()==0\n");
			}*/
			
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

			tfs.push_back(trans_res);
		}

		kd_eth.begin();
		free(pointSet);
		pointSet = NULL;

		if (useControlPoints){
			//transform control points
			for (int i = 0; i < cpoints.size(); i++){
				std::vector<float> rot, trans;
				Transformation2RotTrans(ctfs[i], rot, trans);
				cpoints[i] = TransformPoint(rot, trans, cpoints[i]);
				cnormals[i] = TransformNormal(rot, cnormals[i]);
			}
		}

		//just for debug
		PointsIO::savePLYfile("cpoints.ply", cpoints, cnormals, Vector3u(255, 255, 0));
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