// Hao added it
#include <math.h>

#include "ITMMotionAnalysis.h"
#include "../../Utils/KDtree/kdtree_search_eth.h"

using namespace ITMLib::Engine;

ITMMotionAnalysis::ITMMotionAnalysis(const ITMRGBDCalib *calib){
	this->calib = const_cast<ITMRGBDCalib*>(calib);
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

void ITMMotionAnalysis::getAllSurfacePointsTransformation(const std::vector<std::vector<Vector3f>> &cblocks_p, const std::vector<Vector3f> &cpoints, std::vector<Transformation> &tfs){
	for (int i = 0; i < cblocks_p.size(); i++){
		std::vector<Vector3f> pts = cblocks_p[i];
		for (int j = 0; j < pts.size(); j++){
			tfs.push_back(ctfs[i]);
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
