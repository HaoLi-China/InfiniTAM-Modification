// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"
#include "DeviceAgnostic\ITMRepresentationAccess.h"
#include "../../Utils/PointsIO/PointsIO.h"
//#include "../../Utils/KDtree/MyKDtree.h"
//#include "../../Utils/KDtree/CUDA_KDtree.h"
#include "../../Utils/KDtree/kdtree_search_eth.h"
#include <fstream>

using namespace ITMLib::Engine;

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
	std::cout << "new Frame" << std::endl;
	// prepare image and turn it into a depth image
	if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, settings->modelSensorNoise);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	if (!mainProcessingActive) return;

	if (settings->useMechanicalFusion){
		//init motion analysis
		std::vector<Vector3f> points;
		std::vector<Vector3f> normals;
		std::vector<Vector3f> sur_points;
		std::vector<Vector3f> sur_normals;
		std::vector<short> sdf_s;
		std::vector<uchar> w_s;
		std::vector<Vector3f> ncpoints;
		std::vector<Vector3f> ncnormals;
		std::vector<bool> visiblelist;// visiblelist size = cpoints size
	
		if (settings->useControlPoints){
			getAllOperationPoints(points, normals, sur_points, sur_normals, sdf_s, w_s, true);
			updateControlPoints(sur_points, sur_normals, cpoints, cnormals);
		}
		else{
			getAllOperationPoints(points, normals, sur_points, sur_normals, sdf_s, w_s, true);
		}

		//just for debug
		//PointsIO::savePLYfile("cpoints.ply", cpoints, cnormals);
		//PointsIO::savePLYfile("cpoints.ply", cpoints, cnormals, Vector3u(255, 255, 255));
		if (settings->useControlPoints && cpoints.size() > 0){
			cpoints_vec.push_back(cpoints);
			Vector3f color;
			color[0] = rand()*1.0f / float(RAND_MAX);
			color[1] = rand()*1.0f / float(RAND_MAX);
			color[2] = rand()*1.0f / float(RAND_MAX);
			color_vec.push_back(color);

			getVisibleControlPoints(cpoints, visiblelist);
			motionAnalysis->initialize(cpoints, cnormals, visiblelist);
			////motionAnalysis->optimizeEnergyFunction(view->depth);
			motionAnalysis->optimizeEnergyFunctionNlopt(view->depth);

			//transform
			std::vector<Transformation> tfs;
			motionAnalysis->getAllOperationPointsTransformation(points, cpoints, cnormals, tfs);
			denseMapper->ResetScene(scene);
			transformVoxels(points, sdf_s, w_s, tfs);
		}
		else if (!(settings->useControlPoints) && points.size() > 0){
			getVisiblePoints(sur_points, visiblelist);
			motionAnalysis->initialize(sur_points, sur_normals, visiblelist);
			motionAnalysis->optimizeEnergyFunctionNlopt(view->depth);

			//transform
			std::vector<Transformation> tfs;
			motionAnalysis->getAllOperationPointsTransformation(points, sur_points, sur_normals, tfs);
			denseMapper->ResetScene(scene);
			transformVoxels(points, sdf_s, w_s, tfs);
		}
	}
	else{
		// tracking
		//trackingController->Track(trackingState, view);
	}

	// fusion
	if (fusionActive) denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);
	
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

//Hao added it: get all operation points
void ITMMainEngine::getAllOperationPoints(std::vector<Vector3f> &points, std::vector<Vector3f> &normals, std::vector<Vector3f> &sur_points, std::vector<Vector3f> &sur_normals, std::vector<short> &sdf_s, std::vector<uchar> &w_s, const bool withNormals){
	points.clear();
	normals.clear();
	sur_points.clear();
	sur_normals.clear();
	sdf_s.clear();
	w_s.clear();

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
				if (value<50 * mu&&value>-50 * mu){ //mu=0.02
					//cout<<"value:"<<value<<endl;
					sdf_s.push_back(res.sdf);
					w_s.push_back(res.w_depth);

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

						if (value < 10 * mu && value > -10 * mu){
							sur_normals.push_back(n);
						}
					}

					points.push_back(p);

					if (value < 10 * mu && value > -10 * mu){
						sur_points.push_back(p);
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

//Hao added it: update control points
void ITMMainEngine::updateControlPoints(const std::vector<Vector3f> &sur_points, const std::vector<Vector3f> &sur_normals, std::vector<Vector3f> &cpoints, std::vector<Vector3f> &cnormals){
	
	if (cpoints.size() == 0){
		std::vector<Vector3f> ncpoints;
		std::vector<Vector3f> ncnormals;
		getNewControlPoints(sur_points, sur_normals, ncpoints, ncnormals);

		for (int i = 0; i < ncpoints.size(); i++){
			cpoints.push_back(ncpoints[i]);
			cnormals.push_back(ncnormals[i]);
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

//Hao added it: transform voxels on surface
void ITMMainEngine::transformVoxels(const std::vector<Vector3f> &points, const std::vector<short> &sdf_s, const std::vector<uchar> &w_s, const std::vector<Transformation> &tfs){
	std::vector<Vector3f> trans_points;
	unsigned int hashIdx;

	for (int i = 0; i < points.size(); i++){
		std::vector<float> rot;
		std::vector<float> trans;
		motionAnalysis->Transformation2RotTrans(tfs[i], rot, trans);
		Vector3f pt_tem = motionAnalysis->TransformPoint(rot, trans, points[i]);

		trans_points.push_back(pt_tem);
	}

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

	for (int i = 0; i < trans_points.size(); i++){
		Vector3f pt = trans_points[i];

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

		voxels[(hashTable[hashIdx].ptr * SDF_BLOCK_SIZE3) + lineIndex].sdf = sdf_s[i];
		voxels[(hashTable[hashIdx].ptr * SDF_BLOCK_SIZE3) + lineIndex].w_depth = w_s[i];
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
void ITMMainEngine::getVisibleControlPoints(const std::vector<Vector3f> &cpoints, std::vector<bool> &visiblelist){
	visiblelist.clear();

	for (int i = 0; i < cpoints.size(); i++){
		visiblelist.push_back(false);
	}

	ITMPointCloud *vpointCloud = new ITMPointCloud(trackedImageSize, MEMORYDEVICE_CPU);
	Vector4f *vpoint = vpointCloud->locations->GetData(MEMORYDEVICE_CPU);

	vpointCloud->noTotalPoints = trackedImageSize.x * trackedImageSize.y;

	bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
	flag = true;
#endif
	if (flag){
		ITMSafeCall(cudaMemcpy(vpoint, trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA), trackedImageSize.x * trackedImageSize.y * sizeof(Vector4f), cudaMemcpyDeviceToHost));
	}
	else{
		memcpy(vpoint, trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA), trackedImageSize.x * trackedImageSize.y * sizeof(Vector4f));
	}

	std::vector<Vector3f> vpts;
	for (int i = 0; i < trackedImageSize.x*trackedImageSize.y; i++){
		if (vpoint[i].w > 0){
			Vector3f pt(vpoint[i].x, vpoint[i].y, vpoint[i].z);
			vpts.push_back(pt);
		}
	}

	//just for debug
	const std::vector<Vector3f> normals;
	const std::vector<Vector3u> colors;
	PointsIO::savePLYfile("test.ply", vpts, normals, colors);

	Vector3f *pointSet = (Vector3f*)malloc((vpts.size())*sizeof(Vector3f));
	for (int i = 0; i < vpts.size(); i++){
		pointSet[i].x = vpts[i].x;
		pointSet[i].y = vpts[i].y;
		pointSet[i].z = vpts[i].z;
	}

	KdTreeSearch_ETH kd_eth;
	kd_eth.add_vertex_set(pointSet, vpts.size());
	kd_eth.end();

	float voxelSize = 0.005f;
	std::vector<unsigned int> neighbors;
	for (int i = 0; i < cpoints.size(); i++){
		Vector3f p = cpoints[i];
		std::vector<Vector3f> neighbors;
		std::vector<double> squared_distances;
		//get neighbor points within a range of radius
		kd_eth.find_closest_K_points(p, 1, neighbors, squared_distances);

		if (sqrt(squared_distances[0]) < 2*voxelSize){
			visiblelist[i] = true;
		}
		else{
			visiblelist[i] = false;
		}
	}

	kd_eth.begin();
	free(pointSet);
	pointSet = NULL;

	delete vpointCloud;
	vpointCloud = NULL;
}

//Hao added it: Get Visible Points
void ITMMainEngine::getVisiblePoints(const std::vector<Vector3f> &points, std::vector<bool> &visiblelist){
	visiblelist.clear();

	for (int i = 0; i < points.size(); i++){
		visiblelist.push_back(false);
	}

	ITMPointCloud *vpointCloud = new ITMPointCloud(trackedImageSize, MEMORYDEVICE_CPU);
	Vector4f *vpoint = vpointCloud->locations->GetData(MEMORYDEVICE_CPU);
	
	vpointCloud->noTotalPoints = trackedImageSize.x * trackedImageSize.y;

	bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
	flag = true;
#endif
	if (flag){
		ITMSafeCall(cudaMemcpy(vpoint, trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA), trackedImageSize.x * trackedImageSize.y * sizeof(Vector4f), cudaMemcpyDeviceToHost));
	}
	else{
		memcpy(vpoint, trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA), trackedImageSize.x * trackedImageSize.y * sizeof(Vector4f));
	}

	std::vector<Vector3f> vpts;
	for (int i = 0; i < trackedImageSize.x*trackedImageSize.y; i++){
		if (vpoint[i].w > 0){
			Vector3f pt(vpoint[i].x, vpoint[i].y, vpoint[i].z);
			vpts.push_back(pt);
		}
	}

	Vector3f *pointSet = (Vector3f*)malloc((vpts.size())*sizeof(Vector3f));
	for (int i = 0; i < vpts.size(); i++){
		pointSet[i].x = vpts[i].x;
		pointSet[i].y = vpts[i].y;
		pointSet[i].z = vpts[i].z;
	}

	KdTreeSearch_ETH kd_eth;
	kd_eth.add_vertex_set(pointSet, vpts.size());
	kd_eth.end();

	float voxelSize = 0.005f;
	std::vector<unsigned int> neighbors;
	for (int i = 0; i < points.size(); i++){
		Vector3f p = points[i];
		std::vector<Vector3f> neighbors;
		std::vector<double> squared_distances;
		//get neighbor points within a range of radius
		kd_eth.find_closest_K_points(p, 1, neighbors, squared_distances);

		if (sqrt(squared_distances[0]) < voxelSize){
			visiblelist[i] = true;
		}
		else{
			visiblelist[i] = false;
		}
	}

	kd_eth.begin();
	free(pointSet);
	pointSet = NULL;

	delete vpointCloud;
	vpointCloud = NULL;
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
				if (value<10 * mu&&value>-10 * mu){ //mu=0.02
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

void ITMMainEngine::saveSurfacePoints(const std::string &filename){
	std::vector<Vector3f> points;
	std::vector<Vector3f> normals;
	std::vector<Vector3u> colors;

	getAllSurfacePoints(points, normals, true);
	PointsIO::savePLYfile(filename, points, normals, colors);
}

////Hao added it: reset all voxels
//void ITMMainEngine::resetAllVoxels(){
//	ORUtils::MemoryBlock<ITMHashEntry> *hashEntries = new ORUtils::MemoryBlock<ITMHashEntry>(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE, MEMORYDEVICE_CPU);
//	ITMHashEntry *hashTable = hashEntries->GetData(MEMORYDEVICE_CPU);
//
//	bool flag = false;
//#ifndef COMPILE_WITHOUT_CUDA
//	flag = true;
//#endif
//	if (flag){
//		ITMSafeCall(cudaMemcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
//	}
//	else{
//		memcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry));
//	}
//
//	memset(hashTable, 0, (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(NodeHashEntry));
//	for (int i = 0; i < SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE; i++) {
//		hashTable[i].ptr = -2;
//	}
//
//	if (flag){
//		ITMSafeCall(cudaMemcpy(scene->index.GetEntries(), hashTable, (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry), cudaMemcpyHostToDevice));
//	}
//	else{
//		memcpy(scene->index.GetEntries(), hashTable, (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(ITMHashEntry));
//	}
//
//	free(hashTable);
//	hashTable = NULL;
//}

