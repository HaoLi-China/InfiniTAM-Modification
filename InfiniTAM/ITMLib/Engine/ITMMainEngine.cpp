// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"
#include "DeviceAgnostic\ITMRepresentationAccess.h"
#include "../../Utils/PointsIO/PointsIO.h"
#include "../../Utils/KDtree/kdtree_search_eth.h"
#include "../Utils/ITMRasterization.h"
#include <fstream>

using namespace ITMLib::Engine;
using namespace ITMLib::Utils;

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	static const bool createMeshingEngine = true;

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	this->scene = new ITMScene<ITMVoxel, ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping,
		settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

	meshingEngine = NULL;

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		lowLevelEngine = new ITMLowLevelEngine_CPU();
		viewBuilder = new ITMViewBuilder_CPU(calib);
		visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();
		viewBuilder = new ITMViewBuilder_CUDA(calib);
		visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		lowLevelEngine = new ITMLowLevelEngine_Metal();
		viewBuilder = new ITMViewBuilder_Metal(calib);
		visualisationEngine = new ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	}

	mesh = NULL;
	if (createMeshingEngine) mesh = new ITMMesh(settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

	trackedImageSize = ITMTrackingController::GetTrackedImageSize(settings, imgSize_rgb, imgSize_d);

	renderState_live = visualisationEngine->CreateRenderState(trackedImageSize);
	renderState_freeview = NULL; //will be created by the visualisation engine

	denseMapper = new ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(settings);
	denseMapper->ResetScene(scene);

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>::Instance().Make(trackedImageSize, settings, lowLevelEngine, imuCalibrator, scene);
	trackingController = new ITMTrackingController(tracker, visualisationEngine, lowLevelEngine, settings);

	trackingState = trackingController->BuildTrackingState(trackedImageSize);
	tracker->UpdateInitialPose(trackingState);

	view = NULL; // will be allocated by the view builder

	fusionActive = true;
	mainProcessingActive = true;

	motionAnalysis = new ITMMotionAnalysis(calib, settings->useControlPoints);
}

ITMMainEngine::~ITMMainEngine()
{
	delete renderState_live;
	if (renderState_freeview != NULL) delete renderState_freeview;

	delete scene;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != NULL) delete view;

	delete visualisationEngine;

	if (meshingEngine != NULL) delete meshingEngine;

	if (mesh != NULL) delete mesh;
}

ITMMesh* ITMMainEngine::UpdateMesh(void)
{
	if (mesh != NULL) meshingEngine->MeshScene(mesh, scene);
	return mesh;
}

void ITMMainEngine::SaveSceneToMesh(const char *objFileName)
{
	if (mesh == NULL) return;
	meshingEngine->MeshScene(mesh, scene);
	mesh->WriteSTL(objFileName);
}

//Hao modified it
void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, settings->modelSensorNoise);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	if (!mainProcessingActive) return;

	if (settings->useMechanicalFusion){
		//init motion analysis
		std::vector<Vector3f> voxel_centers;
		std::vector<short> sdf_s;
		std::vector<uchar> w_s;
		std::vector<Vector3f> sur_points;
		std::vector<Vector3f> sur_normals;
		std::vector<Vector3f> ncpoints;
		std::vector<Vector3f> ncnormals;
		std::vector<Vector3f> tcpoints;
		std::vector<Vector3f> tcnormals;
		std::vector<bool> visiblelist;// visiblelist size = cpoints size

		//testSamePosControlPoints(cpoints);//just for debug

		if (settings->useControlPoints && cpoints.size() > 0){
			Vector3f color;
			color[0] = rand()*1.0f / float(RAND_MAX);
			color[1] = rand()*1.0f / float(RAND_MAX);
			color[2] = rand()*1.0f / float(RAND_MAX);
			color_vec.push_back(color);
			printf("aaaaaa\n");
			getTransformedControlPoints(tcpoints, tcnormals);
			printf("bbbbbb\n");
			getVisibleControlPoints(tcpoints, visiblelist);
			printf("cccccc\n");
			cpoints_vec.push_back(tcpoints);

			motionAnalysis->initialize(tcpoints, tcnormals, visiblelist);
			motionAnalysis->optimizeEnergyFunctionNlopt(view->depth);
			
			//transform
			std::vector<Transformation> ctfs;
			motionAnalysis->getAllTransformations(ctfs);
			updateAccumTfs(accumTfs, ctfs);
			ctfs_vec.push_back(ctfs);
			printf("dddddd\n");
		}

		// fusion
		int depth_image_width = view->depth->noDims.x;
		int depth_image_height = view->depth->noDims.y;
		float *depth_device = view->depth->GetData(MEMORYDEVICE_CUDA);
		float *depth = (float*)malloc(depth_image_width*depth_image_height * sizeof(float));
		ITMSafeCall(cudaMemcpy(depth, depth_device, (depth_image_width * depth_image_height)*sizeof(float), cudaMemcpyDeviceToHost));
		printf("eeeeee\n");
		std::vector<Vector3f> live_points;
		getLivePoints(cpoints, accumTfs, depth, live_points);
		printf("ffffff\n");
		allocateNewVoxels(live_points);
		printf("gggggg\n");
		getAllVoxelCenters(voxel_centers, sdf_s, w_s);
		transformAllVoxelCenters(cpoints, accumTfs, voxel_centers);
		integrateIntoCanonicalModel(voxel_centers, depth, sdf_s, w_s);
		printf("hhhhhh\n");
		free(depth);
		depth = NULL;

		//use control nodes
		if (settings->useControlPoints){
			//matching cube to get mesh of canonical model
			UpdateMesh(); printf("iiiiii\n");
			//get surface points of canonical model
			getAllSurfacePoints(sur_points, sur_normals, true); printf("jjjjjj\n");
			//get control points on the surface
			updateControlPoints(sur_points, sur_normals, cpoints, cnormals); printf("kkkkkk\n");
		}
	}
	else{
		// tracking
		trackingController->Track(trackingState, view);
		if (fusionActive) denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);
	}

	// raycast to renderState_live for tracking and free visualisation
	trackingController->Prepare(trackingState, view, renderState_live);
}

Vector2i ITMMainEngine::GetImageSize(void) const
{
	return renderState_live->raycastImage->noDims;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->trackerType == ITMLib::Objects::ITMLibSettings::TRACKER_WICP)
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depthUncertainty->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::WeightToUchar4(out, view->depthUncertainty);
		}
		else
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		}

		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		ORUtils::Image<Vector4u> *srcImage = renderState_live->raycastImage;
		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	{
		IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
		if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
		if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(out->noDims);

		visualisationEngine->FindVisibleBlocks(pose, intrinsics, renderState_freeview);
		visualisationEngine->CreateExpectedDepths(pose, intrinsics, renderState_freeview);
		visualisationEngine->RenderImage(pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	};
}

void ITMMainEngine::turnOnIntegration() { fusionActive = true; }
void ITMMainEngine::turnOffIntegration() { fusionActive = false; }
void ITMMainEngine::turnOnMainProcessing() { mainProcessingActive = true; }
void ITMMainEngine::turnOffMainProcessing() { mainProcessingActive = false; }

//Hao added it: update accumulative transformation
void ITMMainEngine::updateAccumTfs(std::vector<Transformation> &accumTfs, const std::vector<Transformation> &ctfs){
	for (int i = 0; i < ctfs.size(); i++){
		if (i >= accumTfs.size()){
			accumTfs.push_back(ctfs[i]);
		}
		else{
			std::vector<float> global_rot, new_rot;
			std::vector<float> global_trans, new_trans;
			motionAnalysis->Transformation2RotTrans(accumTfs[i], global_rot, global_trans);
			motionAnalysis->Transformation2RotTrans(ctfs[i], new_rot, new_trans);

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

			motionAnalysis->RotTrans2Transformation(global_rot, global_trans, accumTfs[i]);
		}
	}
}

//Hao added it: update control points
void ITMMainEngine::updateControlPoints(const std::vector<Vector3f> &sur_points, const std::vector<Vector3f> &sur_normals, std::vector<Vector3f> &cpoints, std::vector<Vector3f> &cnormals){

	if (cpoints.size() == 0){
		std::vector<Vector3f> ncpoints;
		std::vector<Vector3f> ncnormals;
		getNewControlPoints(sur_points, sur_normals, ncpoints, ncnormals);

		for (int i = 0; i < ncpoints.size(); i++){
			cpoints.push_back(ncpoints[i]);
			cnormals.push_back(ncnormals[i]);

			Transformation trans = { 0, 0, 0, 0, 0, 0 };
			accumTfs.push_back(trans);
		}

		return;
	}

	Vector3f *pointSet = (Vector3f*)malloc(cpoints.size()*sizeof(Vector3f));
	memset(pointSet, 0, cpoints.size()*sizeof(Vector3f));

	for (int i = 0; i < cpoints.size(); i++){
		pointSet[i].x = cpoints[i].x;
		pointSet[i].y = cpoints[i].y;
		pointSet[i].z = cpoints[i].z;
	}

	KdTreeSearch_ETH kd_eth;
	kd_eth.add_vertex_set(pointSet, cpoints.size());
	kd_eth.end();

	std::vector<Vector3f> uspoints;
	std::vector<Vector3f> usnormals;
	for (int i = 0; i < sur_points.size(); i++){
		Vector3f p = sur_points[i];
		std::vector<unsigned int> neighbors;
		std::vector<double> squared_distances;
		kd_eth.find_closest_K_points(p, 1, neighbors, squared_distances);

		if (squared_distances[0] >= CP_RESOLUTION * CP_RESOLUTION){
			uspoints.push_back(sur_points[i]);
			usnormals.push_back(sur_normals[i]);
		}
	}

	kd_eth.begin();
	free(pointSet);
	pointSet = NULL;

	std::vector<Vector3f> ncpoints;
	std::vector<Vector3f> ncnormals;
	getNewControlPoints(uspoints, usnormals, ncpoints, ncnormals);

	std::vector<Transformation> ntfs;
	motionAnalysis->inferTransformations(cpoints, accumTfs, ncpoints, ntfs);

	for (int i = 0; i < ntfs.size(); i++){
		accumTfs.push_back(ntfs[i]);
	}

	for (int i = 0; i < ncpoints.size(); i++){
		cpoints.push_back(ncpoints[i]);
		cnormals.push_back(ncnormals[i]);
	}
}

//Hao added it: get new control points
void ITMMainEngine::getNewControlPoints(const std::vector<Vector3f> &uspoints, const std::vector<Vector3f> &usnormals, std::vector<Vector3f> &ncpoints, std::vector<Vector3f> &ncnormals){
	ncpoints.clear();
	ncnormals.clear();

	Vector3f *pointSet = (Vector3f*)malloc(uspoints.size()*sizeof(Vector3f));
	memset(pointSet, 0, uspoints.size()*sizeof(Vector3f));

	std::vector<bool> conlist;
	for (int i = 0; i < uspoints.size(); i++){
		pointSet[i].x = uspoints[i].x;
		pointSet[i].y = uspoints[i].y;
		pointSet[i].z = uspoints[i].z;

		conlist.push_back(false);
	}

	KdTreeSearch_ETH kd_eth;
	kd_eth.add_vertex_set(pointSet, uspoints.size());
	kd_eth.end();


	for (int i = 0; i < uspoints.size(); i++){
		Vector3f p = uspoints[i];

		std::vector<unsigned int> neighbors;
		kd_eth.find_points_in_radius(p, CP_RESOLUTION * CP_RESOLUTION, neighbors);

		bool flag = false;
		for (int j = 0; j < neighbors.size(); j++){
			int index = neighbors[j];
			flag = (flag || conlist[index]);
		}

		if (!flag){
			conlist[i] = true;
			ncpoints.push_back(uspoints[i]);
			ncnormals.push_back(usnormals[i]);
		}
	}

	kd_eth.begin();
	free(pointSet);
	pointSet = NULL;
}

//transform control points to (k-1)th frame
void ITMMainEngine::getTransformedControlPoints(std::vector<Vector3f> &tcpoints, std::vector<Vector3f> &tcnormals){
	tcpoints.clear();

	for (int i = 0; i < cpoints.size(); i++){
		std::vector<float> rot;
		std::vector<float> trans;
		motionAnalysis->Transformation2RotTrans(accumTfs[i], rot, trans);

		Vector3f p = motionAnalysis->TransformPoint(rot, trans, cpoints[i]);
		Vector3f n = motionAnalysis->TransformNormal(rot, cnormals[i]);

		tcpoints.push_back(p);
		tcnormals.push_back(n);
	}
}

//get all surface points
void ITMMainEngine::getAllSurfacePoints(std::vector<Vector3f> &points, std::vector<Vector3f> &normals, const bool withNormals){
	ORUtils::MemoryBlock<ITMHashEntry> *hashEntries = new ORUtils::MemoryBlock<ITMHashEntry>(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE, MEMORYDEVICE_CPU);
	ITMHashEntry *hashTable = hashEntries->GetData(MEMORYDEVICE_CPU);
	ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));

	bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
	flag = true;
#endif
	if (flag){
		ITMSafeCall(cudaMemcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
	}
	else{
		memcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry));
		memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
	}

	float mu = scene->sceneParams->mu;

	for (int i = 0; i < SDF_BUCKET_NUM * 1 + SDF_EXCESS_LIST_SIZE; i++){
		const ITMHashEntry &hashEntry = hashTable[i];

		if (hashEntry.ptr >= 0){
			for (int j = 0; j < SDF_BLOCK_SIZE3; j++){
				ITMVoxel res = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];

				float value = ITMVoxel::SDF_valueToFloat(res.sdf);
				if (value<5 * mu&&value>-5 * mu){ //mu=0.02
					Vector3f p;
					float voxelSize = 0.125f;
					float blockSizeWorld = scene->sceneParams->voxelSize*SDF_BLOCK_SIZE; // = 0.005*8;

					p.z = (hashEntry.pos.z + (j / (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE) + 0.5f)*voxelSize)*blockSizeWorld;
					p.y = (hashEntry.pos.y + ((j % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE + 0.5f)*voxelSize)*blockSizeWorld;
					p.x = (hashEntry.pos.x + ((j % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) % SDF_BLOCK_SIZE + 0.5f)*voxelSize)*blockSizeWorld;

					if (withNormals){
						Vector3f n;

						Vector3f pt((hashEntry.pos.x*SDF_BLOCK_SIZE + ((j % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) % SDF_BLOCK_SIZE + 0.5f)), (hashEntry.pos.y*SDF_BLOCK_SIZE + ((j % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE + 0.5f)), (hashEntry.pos.z*SDF_BLOCK_SIZE + (j / (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE) + 0.5f)));

						Vector3f normal_host = computeSingleNormalFromSDF(voxels, hashTable, pt);

						float normScale = 1.0f / sqrtf(normal_host.x * normal_host.x + normal_host.y * normal_host.y + normal_host.z * normal_host.z);
						normal_host *= normScale;

						Vector3f pn(normal_host[0], normal_host[1], normal_host[2]);
						Vector3f tem(-p.x, -p.y, -p.z);

						double dotvalue = pn.x*tem.x + pn.y*tem.y + pn.z*tem.z;
						if (dotvalue < 0){
							pn = -pn;
						}

						n.x = pn.x;
						n.y = pn.y;
						n.z = pn.z;

						normals.push_back(n);
					}

					points.push_back(p);
				}
			}
		}
	}

	free(voxels);
	free(hashTable);
	voxels = NULL;
	hashTable = NULL;
}


//get all voxel centers
void ITMMainEngine::getAllVoxelCenters(std::vector<Vector3f> &points, std::vector<short> &sdf_s, std::vector<uchar> &w_s){
	ORUtils::MemoryBlock<ITMHashEntry> *hashEntries = new ORUtils::MemoryBlock<ITMHashEntry>(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE, MEMORYDEVICE_CPU);
	ITMHashEntry *hashTable = hashEntries->GetData(MEMORYDEVICE_CPU);
	ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));

	bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
	flag = true;
#endif
	if (flag){
		ITMSafeCall(cudaMemcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
	}
	else{
		memcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry));
		memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
	}

	float mu = scene->sceneParams->mu;

	for (int i = 0; i < SDF_BUCKET_NUM * 1 + SDF_EXCESS_LIST_SIZE; i++){
		const ITMHashEntry &hashEntry = hashTable[i];

		if (hashEntry.ptr >= 0){
			for (int j = 0; j < SDF_BLOCK_SIZE3; j++){
				ITMVoxel res = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];

				float value = ITMVoxel::SDF_valueToFloat(res.sdf);
				if (value<=50 * mu&&value>=-50 * mu){ //mu=0.02
					Vector3f p;
					float voxelSize = 0.125f;
					float blockSizeWorld = scene->sceneParams->voxelSize*SDF_BLOCK_SIZE; // = 0.005*8;

					p.z = (hashEntry.pos.z + (j / (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE) + 0.5f)*voxelSize)*blockSizeWorld;
					p.y = (hashEntry.pos.y + ((j % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE + 0.5f)*voxelSize)*blockSizeWorld;
					p.x = (hashEntry.pos.x + ((j % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) % SDF_BLOCK_SIZE + 0.5f)*voxelSize)*blockSizeWorld;

					points.push_back(p);
					sdf_s.push_back(res.sdf);
					w_s.push_back(res.w_depth);
				}
			}
		}
	}

	free(voxels);
	free(hashTable);
	voxels = NULL;
	hashTable = NULL;
}

//transform all voxel centers
void ITMMainEngine::transformAllVoxelCenters(const std::vector<Vector3f> &cpoints, const std::vector<Transformation> &accumTfs, std::vector<Vector3f> &points){
	if (cpoints.size() == 0){
		return;
	}
	
	std::vector<Transformation> ntfs;
	motionAnalysis->inferTransformations(cpoints, accumTfs, points, ntfs);

	for (int i = 0; i < ntfs.size(); i++){
		std::vector<float> rot, trans;
		motionAnalysis->Transformation2RotTrans(ntfs[i], rot, trans);
		points[i] = motionAnalysis->TransformPoint(rot, trans, points[i]);
	}
}

//integrate into canonical model
void ITMMainEngine::integrateIntoCanonicalModel(const std::vector<Vector3f> &points, const float* depth, std::vector<short> &sdf_s, std::vector<uchar> &w_s){
	ITMRGBDCalib *calib = NULL;
	motionAnalysis->getCalib(calib);
	Vector4f projParams_d = calib->intrinsics_d.projectionParamsSimple.all;
	Vector2i imgSize(trackedImageSize.x, trackedImageSize.y);

	//update sdf and weight
	for (int i = 0; i < points.size(); i++){
		Vector3f pt_camera = points[i];
		Vector2f pt_image;
		float depth_measure, eta, oldF, newF;
		int oldW, newW;
		float mu = scene->sceneParams->mu;

		// project point into image
		if (pt_camera.z <= 0) continue;

		pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
		pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
		if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) continue;

		// get measured depth from image
		depth_measure = depth[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x];
		if (depth_measure <= 0.0) continue;

		// check whether voxel needs updating
		eta = depth_measure - pt_camera.z;
		if (eta < -mu) continue;

		// compute updated SDF value and reliability
		oldF = ITMVoxel::SDF_valueToFloat(sdf_s[i]); oldW = w_s[i];

		newF = MIN(1.0f, eta / mu);
		newW = 1;

		newF = oldW * oldF + newW * newF;
		newW = oldW + newW;
		newF /= newW;
		//newW = MIN(newW, maxW);

		// write back
		sdf_s[i] = ITMVoxel::SDF_floatToValue(newF);
		w_s[i] = newW;
	}

	//update canonical model
	ORUtils::MemoryBlock<ITMHashEntry> *hashEntries = new ORUtils::MemoryBlock<ITMHashEntry>(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE, MEMORYDEVICE_CPU);
	ITMHashEntry *hashTable = hashEntries->GetData(MEMORYDEVICE_CPU);
	ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));

	bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
	flag = true;
#endif
	if (flag){
		ITMSafeCall(cudaMemcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
	}
	else{
		memcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry));
		memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
	}

	float mu = scene->sceneParams->mu;
	int count = 0;

	for (int i = 0; i < SDF_BUCKET_NUM * 1 + SDF_EXCESS_LIST_SIZE; i++){
		const ITMHashEntry &hashEntry = hashTable[i];

		if (hashEntry.ptr >= 0){
			for (int j = 0; j < SDF_BLOCK_SIZE3; j++){
				ITMVoxel res = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];
				float value = ITMVoxel::SDF_valueToFloat(res.sdf);

				if (value<=50 * mu&&value>=-50 * mu){ //mu=0.02
					voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].sdf = sdf_s[count];
					voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].w_depth = w_s[count];

					count++;
				}
			}
		}
	}

	if (flag){
		ITMSafeCall(cudaMemcpy(scene->index.GetEntries(), hashTable, (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry), cudaMemcpyHostToDevice));
		ITMSafeCall(cudaMemcpy(scene->localVBA.GetVoxelBlocks(), voxels, SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyHostToDevice));
	}
	else{
		memcpy(scene->index.GetEntries(), hashTable, (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry));
		memcpy(scene->localVBA.GetVoxelBlocks(), voxels, SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel));
	}

	free(voxels);
	free(hashTable);
	voxels = NULL;
	hashTable = NULL;
}

//get live points
void ITMMainEngine::getLivePoints(const std::vector<Vector3f> &cpoints, const std::vector<Transformation> &accumTfs, const float* depth, std::vector<Vector3f> &live_points){
	live_points.clear();

	ITMRGBDCalib *calib = NULL;
	motionAnalysis->getCalib(calib);
	Vector4f projParams_d = calib->intrinsics_d.projectionParamsSimple.all;
	Vector2i imgSize(trackedImageSize.x, trackedImageSize.y);

	for (int y = 0; y < imgSize.y; y++){
		for (int x = 0; x < imgSize.x; x++){
			int id = imgSize.x * y + x;
			if (depth[id] > 0){
				Vector3f pt;
				pt.z = depth[id];
				pt.x = pt.z * ((float(x) - projParams_d.z) * 1.0 / projParams_d.x);
				pt.y = pt.z * ((float(y) - projParams_d.w) * 1.0 / projParams_d.y);

				live_points.push_back(pt);
			}
		}
	}

	const std::vector<Vector3f> nors;
	PointsIO::savePLYfile("live_pointsa.ply", live_points, nors, Vector3u(255, 0, 0));

	if (cpoints.size() == 0){
		return;
	}

	std::vector<Transformation> ntfs;
	motionAnalysis->inferTransformations(cpoints, accumTfs, live_points, ntfs);

	for (int i = 0; i < ntfs.size(); i++){
		std::vector<float> source_rot, source_trans;
		motionAnalysis->Transformation2RotTrans(ntfs[i], source_rot, source_trans);
		std::vector<float> target_rot, target_trans;
		motionAnalysis->invTransformation(source_rot, source_trans, target_rot, target_trans);

		live_points[i] = motionAnalysis->TransformNormal(target_rot, live_points[i] + Vector3f(target_trans[0], target_trans[1], target_trans[2]));
	}

	PointsIO::savePLYfile("live_pointsb.ply", live_points, nors, Vector3u(255, 0, 0));
}

//allocate new voxels
void ITMMainEngine::allocateNewVoxels(const std::vector<Vector3f> &live_points){
	unsigned int hashIdx;

	ORUtils::MemoryBlock<ITMHashEntry> *hashEntries = new ORUtils::MemoryBlock<ITMHashEntry>(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE, MEMORYDEVICE_CPU);
	ITMHashEntry *hashTable = hashEntries->GetData(MEMORYDEVICE_CPU);
	ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
	int *voxelAllocationList = (int*)malloc(SDF_LOCAL_BLOCK_NUM * sizeof(int));
	int *excessAllocationList = (int*)malloc(SDF_EXCESS_LIST_SIZE * sizeof(int));

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState_live;
	int *visibleEntryIDs = (int*)malloc(SDF_LOCAL_BLOCK_NUM * sizeof(int));
	uchar *entriesVisibleType = (uchar*)malloc((SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) * sizeof(uchar));

	int noVisibleEntries = 0;
	int noTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;

	memset(visibleEntryIDs, 0, SDF_LOCAL_BLOCK_NUM * sizeof(int));
	memset(entriesVisibleType, 0, (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) * sizeof(uchar));

	bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
	flag = true;
#endif
	if (flag){
		ITMSafeCall(cudaMemcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(visibleEntryIDs, renderState_vh->GetVisibleEntryIDs(), SDF_LOCAL_BLOCK_NUM * sizeof(int), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(entriesVisibleType, renderState_vh->GetEntriesVisibleType(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) * sizeof(uchar), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(voxelAllocationList, scene->localVBA.GetAllocationList(), SDF_LOCAL_BLOCK_NUM * sizeof(int), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(excessAllocationList, scene->index.GetExcessAllocationList(), SDF_EXCESS_LIST_SIZE * sizeof(int), cudaMemcpyDeviceToHost));
	}
	else{
		memcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry));
		memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
		memcpy(voxelAllocationList, scene->localVBA.GetAllocationList(), SDF_LOCAL_BLOCK_NUM * sizeof(int));
		memcpy(excessAllocationList, scene->index.GetExcessAllocationList(), SDF_EXCESS_LIST_SIZE * sizeof(int));
	}

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();

	for (int i = 0; i < renderState_vh->noVisibleEntries; i++)
		entriesVisibleType[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed

	for (int i = 0; i < live_points.size(); i++){
		Vector3f pt = live_points[i];

		float voxelSize = 0.125f;
		float blockSizeWorld = scene->sceneParams->voxelSize*SDF_BLOCK_SIZE; // = 0.005*8;

		Vector3s blockPos;
		blockPos.x = floor(pt.x / blockSizeWorld);
		blockPos.y = floor(pt.y / blockSizeWorld);
		blockPos.z = floor(pt.z / blockSizeWorld);

		Vector3s voxelPoseInBlock;
		voxelPoseInBlock.x = floor((pt.x / blockSizeWorld - blockPos.x) / voxelSize);
		voxelPoseInBlock.y = floor((pt.y / blockSizeWorld - blockPos.y) / voxelSize);
		voxelPoseInBlock.z = floor((pt.z / blockSizeWorld - blockPos.z) / voxelSize);

		int lineIndex = voxelPoseInBlock.x + voxelPoseInBlock.y*SDF_BLOCK_SIZE + voxelPoseInBlock.z*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;

		//compute index in hash table
		hashIdx = hashIndex(blockPos);

		//check if hash table contains entry
		bool isFound = false;

		ITMHashEntry hashEntry = hashTable[hashIdx];
		unsigned char hashChangeType = 0;

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
		{
			//entry has been streamed out but is visible or in memory and visible
			entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

			isFound = true;
		}

		if (!isFound)
		{
			bool isExcess = false;
			if (hashEntry.ptr >= -1) //seach excess list only if there is no room in ordered part
			{
				while (hashEntry.offset >= 1)
				{
					hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
					hashEntry = hashTable[hashIdx];

					if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
					{
						//entry has been streamed out but is visible or in memory and visible
						entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

						isFound = true;
						break;
					}
				}

				isExcess = true;
			}

			if (!isFound) //still not found
			{
				hashChangeType = isExcess ? 2 : 1; //needs allocation 
				entriesVisibleType[hashIdx] = 1; //new entry is visible
			}
		}

		int vbaIdx, exlIdx;

		switch (hashChangeType)
		{
		case 1: //needs allocation, fits in the ordered list
			vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;

			if (vbaIdx >= 0) //there is room in the voxel block array
			{
				ITMHashEntry hashEntry;
				hashEntry.pos.x = blockPos.x; hashEntry.pos.y = blockPos.y; hashEntry.pos.z = blockPos.z;
				hashEntry.ptr = voxelAllocationList[vbaIdx];
				hashEntry.offset = 0;

				hashTable[hashIdx] = hashEntry;
			}

			break;
		case 2: //needs allocation in the excess list
			vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
			exlIdx = lastFreeExcessListId; lastFreeExcessListId--;

			if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
			{
				ITMHashEntry hashEntry;
				hashEntry.pos.x = blockPos.x; hashEntry.pos.y = blockPos.y; hashEntry.pos.z = blockPos.z;
				hashEntry.ptr = voxelAllocationList[vbaIdx];
				hashEntry.offset = 0;

				int exlOffset = excessAllocationList[exlIdx];

				hashTable[hashIdx].offset = exlOffset + 1; //connect to child

				hashIdx = SDF_BUCKET_NUM + exlOffset;
				hashTable[hashIdx] = hashEntry; //add child to the excess list

				entriesVisibleType[hashIdx] = 1; //make child visible and in memory
			}

			break;
		}

		//voxels[(hashTable[hashIdx].ptr * SDF_BLOCK_SIZE3) + lineIndex].sdf = 0;
		//voxels[(hashTable[hashIdx].ptr * SDF_BLOCK_SIZE3) + lineIndex].w_depth = 1;
	}

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		unsigned char hashVisibleType = entriesVisibleType[targetIdx];
		if (hashVisibleType > 0)
		{
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			noVisibleEntries++;
		}
	}

	renderState_vh->noVisibleEntries = noVisibleEntries;
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);

	if (flag){
		ITMSafeCall(cudaMemcpy(scene->index.GetEntries(), hashTable, (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry), cudaMemcpyHostToDevice));
		ITMSafeCall(cudaMemcpy(scene->localVBA.GetVoxelBlocks(), voxels, SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyHostToDevice));
		ITMSafeCall(cudaMemcpy(renderState_vh->GetVisibleEntryIDs(), visibleEntryIDs, SDF_LOCAL_BLOCK_NUM * sizeof(int), cudaMemcpyHostToDevice));
		ITMSafeCall(cudaMemcpy(renderState_vh->GetEntriesVisibleType(), entriesVisibleType, (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) * sizeof(uchar), cudaMemcpyHostToDevice));
	}
	else{
		memcpy(scene->index.GetEntries(), hashTable, (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry));
		memcpy(scene->localVBA.GetVoxelBlocks(), voxels, SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel));
		memcpy(renderState_vh->GetVisibleEntryIDs(), visibleEntryIDs, SDF_LOCAL_BLOCK_NUM * sizeof(int));
		memcpy(renderState_vh->GetEntriesVisibleType(), entriesVisibleType, (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) * sizeof(uchar));
	}

	free(voxels);
	free(hashTable);
	free(voxelAllocationList);
	free(excessAllocationList);
	free(visibleEntryIDs);
	free(entriesVisibleType);
	voxels = NULL;
	hashTable = NULL;
	voxelAllocationList = NULL;
	excessAllocationList = NULL;
	visibleEntryIDs = NULL;
	entriesVisibleType = NULL;
}

//Hao added it: Get Visible Control Points
void ITMMainEngine::getVisibleControlPoints(const std::vector<Vector3f> &tcpoints, std::vector<bool> &visiblelist){
	visiblelist.clear();

	ITMRGBDCalib *calib = NULL;
	motionAnalysis->getCalib(calib);
	Vector4f projParams_d = calib->intrinsics_d.projectionParamsSimple.all;
	Vector2i imgSize(trackedImageSize.x, trackedImageSize.y);

	ITMRasterization rasterization(calib, trackedImageSize);
	ITMMesh::Triangle *triangles = NULL;
	mesh->getCpuTriangles(triangles);
	int noTriangles = mesh->noTotalTriangles;
	printf("111111\n");
	std::vector<Vector3f> allTriPoints;
	for (int i = 0; i < noTriangles; i++){
		allTriPoints.push_back(triangles[i].p0);
		allTriPoints.push_back(triangles[i].p1);
		allTriPoints.push_back(triangles[i].p2);
	}

	std::vector<Transformation> ntfs;
	motionAnalysis->inferTransformations(cpoints, accumTfs, allTriPoints, ntfs);
	printf("222222\n");
	for (int i = 0; i < ntfs.size(); i++){
		std::vector<float> rot, trans;
		motionAnalysis->Transformation2RotTrans(ntfs[i], rot, trans);

		allTriPoints[i] = motionAnalysis->TransformPoint(rot, trans, allTriPoints[i]);
	}
	printf("333333\n");
	for (int i = 0; i < noTriangles; i++){
		triangles[i].p0 = allTriPoints[3 * i];
		triangles[i].p1 = allTriPoints[3 * i + 1];
		triangles[i].p2 = allTriPoints[3 * i + 2];
	}
	printf("444444\n");
	float *depthImage = NULL;
	rasterization.render(triangles, noTriangles);
	rasterization.getDepthImage(depthImage);
	printf("555555\n");
	free(triangles);
	triangles = NULL;

	for (int i = 0; i < tcpoints.size(); i++){
		visiblelist.push_back(false);
	}

	////debug
	//int count1 = 0;
	//for (int i = 0; i < imgSize.x*imgSize.y; i++){
	//	if (depthImage[i] == 0){
	//		count1++;
	//	}
	//}
	//std::cout << "count1:" << count1 << std::endl;

	std::vector<Vector3f> tem_pts;
	for (int y = 0; y < imgSize.y; y++){
		for (int x = 0; x < imgSize.x; x++){
			int id = imgSize.x * y + x;
			if (depthImage[id] > 0){
				Vector3f pt;
				pt.z = depthImage[id];
				pt.x = pt.z * ((float(x) - projParams_d.z) * 1.0 / projParams_d.x);
				pt.y = pt.z * ((float(y) - projParams_d.w) * 1.0 / projParams_d.y);

				tem_pts.push_back(pt);
			}
		}
	}
	const std::vector<Vector3f> nors;
	PointsIO::savePLYfile("tem_points.ply", tem_pts, nors, Vector3u(0, 0, 255));

	for (int i = 0; i < tcpoints.size(); i++){
		Vector3f pt_camera = tcpoints[i];
		Vector2f pt_image;

		// project point into image
		if (pt_camera.z <= 0) continue;

		pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
		pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;

		int x = (int)(pt_image.x + 0.5f);
		int y = (int)(pt_image.y + 0.5f);

		if ((x < 0) || (x > imgSize.x - 1) || (y < 1) || (y > imgSize.y - 1)) continue;

		std::vector<Vector2i> pos_s;
		findNeighborsInDepthMap(x, y, 3, pos_s);

		for (int k = 0; k < pos_s.size(); k++){
			int index = pos_s[k].y * imgSize.x + pos_s[k].x;
			if (std::abs(depthImage[index] - pt_camera.z) < 2*scene->sceneParams->voxelSize){
				visiblelist[i] = true;
				break;
			}
		}
	}
	
	std::vector<Vector3f> vpts;
	std::vector<Vector3f> nvpts;//debug
	for (int i = 0; i < tcpoints.size(); i++){
		if (visiblelist[i]){
			vpts.push_back(tcpoints[i]);
		}
		else{
			nvpts.push_back(tcpoints[i]);
		}
	}
	printf("666666\n");
	//const std::vector<Vector3f> nors;
	PointsIO::savePLYfile("tcpoints.ply", tcpoints, nors, Vector3u(0, 0, 255));
	PointsIO::savePLYfile("vpts.ply", vpts, nors, Vector3u(0, 255, 0));
	PointsIO::savePLYfile("nvpts.ply", nvpts, nors, Vector3u(255, 0, 0));
}

//just for debug
void ITMMainEngine::saveHashTable(const std::string &filename){
	ORUtils::MemoryBlock<ITMHashEntry> *hashEntries = new ORUtils::MemoryBlock<ITMHashEntry>(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE, MEMORYDEVICE_CPU);
	ITMHashEntry *hashTable = hashEntries->GetData(MEMORYDEVICE_CPU);

	bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
	flag = true;
#endif
	if (flag){
		ITMSafeCall(cudaMemcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
	}
	else{
		memcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry));
	}

	float mu = scene->sceneParams->mu;

	std::ofstream ofs(filename.c_str());

	for (int i = 0; i < SDF_BUCKET_NUM * 1 + SDF_EXCESS_LIST_SIZE; i++){
		const ITMHashEntry &hashEntry = hashTable[i];

		if (hashEntry.ptr >= 0){
			ofs << i << ":" << hashEntry.pos[0] << " " << hashEntry.pos[1] << " " << hashEntry.pos[2] << "\n";
		}
	}

	ofs.close();
	free(hashTable);
	hashTable = NULL;
}

//just for debug
void ITMMainEngine::saveVisibleEntryIDs(const std::string &filename){
	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState_live;
	int *visibleEntryIDs = (int*)malloc(SDF_LOCAL_BLOCK_NUM * sizeof(int));

	int noVisibleEntries = renderState_vh->noVisibleEntries;

	bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
	flag = true;
#endif
	if (flag){
		ITMSafeCall(cudaMemcpy(visibleEntryIDs, renderState_vh->GetVisibleEntryIDs(), SDF_LOCAL_BLOCK_NUM * sizeof(int), cudaMemcpyDeviceToHost));
	}
	else{
		memcpy(visibleEntryIDs, renderState_vh->GetVisibleEntryIDs(), SDF_LOCAL_BLOCK_NUM * sizeof(int));
	}

	std::ofstream ofs(filename.c_str());

	//build visible list
	for (int targetIdx = 0; targetIdx < noVisibleEntries; targetIdx++)
	{
		ofs << targetIdx << ": " << visibleEntryIDs[targetIdx] << "\n";
	}

	ofs.close();

	free(visibleEntryIDs);
	visibleEntryIDs = NULL;
}

//just for debug
void ITMMainEngine::saveEntriesVisibleType(const std::string &filename){
	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState_live;

	int noTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
	uchar *entriesVisibleType = (uchar*)malloc(noTotalEntries* sizeof(uchar));

	bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
	flag = true;
#endif
	if (flag){
		ITMSafeCall(cudaMemcpy(entriesVisibleType, renderState_vh->GetEntriesVisibleType(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) * sizeof(uchar), cudaMemcpyDeviceToHost));
	}
	else{
		memcpy(entriesVisibleType, renderState_vh->GetEntriesVisibleType(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) * sizeof(uchar));
	}

	std::ofstream ofs(filename.c_str());

	//build visible list
	for (int i = 0; i < noTotalEntries; i++)
	{
		if (entriesVisibleType[i]>0){
			ofs << i << ": " << (int)(entriesVisibleType[i]) << "\n";
		}
	}

	ofs.close();

	free(entriesVisibleType);
	entriesVisibleType = NULL;
}

//just for debug
void ITMMainEngine::saveSDFs(const ITMScene<ITMVoxel, ITMVoxelIndex> *scene, const ITMRenderState *renderState, const std::string &filename){
	ORUtils::MemoryBlock<ITMHashEntry> *hashEntries = new ORUtils::MemoryBlock<ITMHashEntry>(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE, MEMORYDEVICE_CPU);
	ITMHashEntry *hashTable = hashEntries->GetData(MEMORYDEVICE_CPU);
	ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));

	bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
	flag = true;
#endif
	if (flag){
		ITMSafeCall(cudaMemcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
	}
	else{
		memcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry));
		memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
	}

	float mu = scene->sceneParams->mu;

	int count = 0;

	std::ofstream ofs(filename.c_str());

	for (int i = 0; i < SDF_BUCKET_NUM * 1 + SDF_EXCESS_LIST_SIZE; i++){
		const ITMHashEntry &hashEntry = hashTable[i];
		if (hashEntry.ptr >= 0){
			std::vector<Vector3f> pts0;
			std::vector<short> sdf0;

			for (int a = 0; a < SDF_BLOCK_SIZE; a++){
				for (int b = 0; b < SDF_BLOCK_SIZE; b++){
					for (int c = 0; c < SDF_BLOCK_SIZE; c++){
						int j = a*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE + b*SDF_BLOCK_SIZE + c;
						ITMVoxel res = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];
						ofs << "i j: " << res.sdf << "\n";
					}
				}
			}
		}
	}

	free(voxels);
	free(hashTable);
	voxels = NULL;
	hashTable = NULL;
}

//just for debug
void ITMMainEngine::testSamePosControlPoints(const std::vector<Vector3f> &cpoints){

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
		std::vector<double> squared_distances;
		std::vector<Vector3f> neighbors;
		//get neighbor points within a range of radius
		kd_eth.find_closest_K_points(p, 2, neighbors, squared_distances);

		if (sqrt(squared_distances[1]) == 0){
			printf("SamePosControlPoints!\n");
		}
	}

	kd_eth.begin();
	free(pointSet);
	pointSet = NULL;
}

void ITMMainEngine::saveSurfacePoints(const std::string &filename){
	std::vector<Vector3f> points;
	std::vector<Vector3f> normals;
	std::vector<Vector3u> colors;

	getAllSurfacePoints(points, normals, true);
	PointsIO::savePLYfile(filename, points, normals, colors);
}

void ITMMainEngine::findNeighborsInDepthMap(int x, int y, int scale, std::vector<Vector2i> &pos_s){
	pos_s.clear();

	int span = scale / 2;

	if (span < 0){
		return;
	}

	for (int i = x - span; i < x + span; i++){
		for (int j = y - span; j < y + span; j++){
			if (i > 0 && i < trackedImageSize.x - 1 && j > 0 && j < trackedImageSize.y - 1){
				Vector2i position(i, j);
				pos_s.push_back(position);
			}
		}
	}
}

