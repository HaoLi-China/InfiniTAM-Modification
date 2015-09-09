// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"
#include "DeviceAgnostic\ITMRepresentationAccess.h"

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

	Vector2i trackedImageSize = ITMTrackingController::GetTrackedImageSize(settings, imgSize_rgb, imgSize_d);

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

	motionAnalysis = new ITMMotionAnalysis(settings, calib, false);
}

ITMMainEngine::~ITMMainEngine()
{
	delete renderState_live;
	if (renderState_freeview!=NULL) delete renderState_freeview;

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
	if (imuMeasurement==NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter,settings->modelSensorNoise);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	if (!mainProcessingActive) return;

	// tracking
	//trackingController->Track(trackingState, view);

	//init motion analysis
	std::vector<Vector3f> points;
	std::vector<Vector3f> normals;
	std::vector<short> sdf_s;
	getSurfacePoints(points, normals, sdf_s, false, true);
	motionAnalysis->setAllNodeinfo(points);

	//transform
	std::vector<Transformation> tfs;
	motionAnalysis->optimizeEnergyFunction(trackingState->pointCloud, view->depth);
	motionAnalysis->getAllTransformation(points, tfs);
	denseMapper->ResetScene(scene);
	transformVoxels(points, sdf_s, tfs);

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
		if (settings->trackerType==ITMLib::Objects::ITMLibSettings::TRACKER_WICP)
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

//Hao added it: get all surface points
void ITMMainEngine::getSurfacePoints(std::vector<Vector3f> &points, std::vector<Vector3f> &normals, std::vector<short> &sdf_s, const bool withNormals, const bool withSDFs){
	points.clear();
	normals.clear();

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
					//cout<<"value:"<<value<<endl;
					if (withSDFs){
						sdf_s.push_back(res.sdf);
					}

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

//Hao added it: transform voxels on surface
void ITMMainEngine::transformVoxels(const std::vector<Vector3f> &points, const std::vector<short> &sdf_s, const std::vector<Transformation> &tfs){
	std::vector<Vector3f> trans_points;
	unsigned int hashIdx;

	for (int i = 0; i < points.size(); i++){
		Vector3f pt_tem;
		Matrix4f mtx;
		motionAnalysis->Transformation2Matrix4(tfs[i], mtx);
		pt_tem = mtx*points[i];

		trans_points.push_back(pt_tem);
	}

	ORUtils::MemoryBlock<ITMHashEntry> *hashEntries = new ORUtils::MemoryBlock<ITMHashEntry>(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE, MEMORYDEVICE_CPU);
	ITMHashEntry *hashTable = hashEntries->GetData(MEMORYDEVICE_CPU);
	ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
	int *voxelAllocationList = (int*)malloc(SDF_LOCAL_BLOCK_NUM * sizeof(int));
	int *excessAllocationList = (int*)malloc(SDF_EXCESS_LIST_SIZE * sizeof(int));

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

		ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState_live;

		//compute index in hash table
		hashIdx = hashIndex(blockPos);

		//check if hash table contains entry
		bool isFound = false;

		ITMHashEntry hashEntry = hashTable[hashIdx];
		unsigned char hashChangeType = 0;

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
		{
			hashChangeType = (hashEntry.ptr == -1) ? 2 : 1;
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
						hashChangeType = (hashEntry.ptr == -1) ? 2 : 1;

						isFound = true;
						break;
					}
				}

				isExcess = true;
			}

			if (!isFound) //still not found
			{
				hashChangeType = isExcess ? 2 : 1; //needs allocation 
			}

			int vbaIdx, exlIdx;

			switch (hashChangeType)
			{
			case 1: //needs allocation, fits in the ordered list
				vbaIdx = lastFreeVoxelBlockId; 
				lastFreeVoxelBlockId--;

				if (vbaIdx >= 0) //there is room in the voxel block array
				{
					Vector4s pt_block_all(blockPos.x, blockPos.y, blockPos.z, 1);

					ITMHashEntry hashEntry;
					hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
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
					Vector4s pt_block_all = (blockPos.x, blockPos.y, blockPos.z, 1);

					ITMHashEntry hashEntry;
					hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					int exlOffset = excessAllocationList[exlIdx];

					hashTable[hashIdx].offset = exlOffset + 1; //connect to child

					hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list
				}

				break;
			}
		}

		voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + lineIndex].sdf = sdf_s[i];
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
	free(voxelAllocationList);
	free(excessAllocationList);
	voxels = NULL;
	hashTable = NULL;
	voxelAllocationList = NULL;
	excessAllocationList = NULL;
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

