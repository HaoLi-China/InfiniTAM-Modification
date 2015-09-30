// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib/Engine/ITMMainEngine.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../Utils/FileUtils.h"
#include "../Utils/NVTimer.h"

#include "ImageSourceEngine.h"
#include "IMUSourceEngine.h"

#include <vector>

namespace InfiniTAM
{
	namespace Engine
	{
		class UIEngine
		{
			static UIEngine* instance;

			enum MainLoopAction
			{
				PROCESS_PAUSED, PROCESS_FRAME, PROCESS_VIDEO, EXIT, SAVE_TO_DISK
			}mainLoopAction;

			struct UIColourMode {
				const char *name;
				ITMMainEngine::GetImageType type;
				UIColourMode(const char *_name, ITMMainEngine::GetImageType _type)
				 : name(_name), type(_type)
				{}
			};
			std::vector<UIColourMode> colourModes;
			int currentColourMode;

			ITMLibSettings internalSettings;
			ImageSourceEngine *imageSource;
			IMUSourceEngine *imuSource;
			ITMMainEngine *mainEngine;

			StopWatchInterface *timer_instant;
			StopWatchInterface *timer_average;

		private: // For UI layout
			static const int NUM_SUB_WIN = 2;
			int subwin[NUM_SUB_WIN];
			static const int NUM_WIN = 3;
			Vector4f winReg[NUM_WIN]; // (x1, y1, x2, y2)
			Vector2i mainWinSize;
			Vector2i subWin1Size;
			Vector2i subWin2Size;
			uint textureId[NUM_WIN];
			ITMUChar4Image *outImage[NUM_WIN];
			ITMMainEngine::GetImageType outImageType[NUM_WIN];

			ITMUChar4Image *inputRGBImage; ITMShortImage *inputRawDepthImage;
			ITMIMUMeasurement *inputIMUMeasurement;

			bool freeviewActive;
			bool intergrationActive;
			ITMPose freeviewPose;
			ITMIntrinsics freeviewIntrinsics;

			int mouseState;
			Vector2i mouseLastClick;

			int currentFrameNo; bool isRecording;

			float thetaX;
			float thetaY;
			float scaleFactor;

			float dx;
			float dy;
			float oldy;
			float oldx;

			float centerX;
			float centerY;
			float centerZ;

			bool isRotating;
			bool isTranslating;
			bool isScaling;
		public:
			static UIEngine* Instance(void) {
				if (instance == NULL) instance = new UIEngine();
				return instance;
			}

			static void glutDisplayFunction();
			static void glutDisplayTrajectoryFunction();
			static void glutIdleFunction();
			static void glutKeyUpFunction(unsigned char key, int x, int y);
			static void glutMouseButtonFunction(int button, int state, int x, int y);
			static void glutMouseMoveFunction(int x, int y);
			static void glutMouseWheelFunction(int button, int dir, int x, int y);
			static void glutReshapeTrajectoryView(int width, int height);
			static void glutTrajectoryViewMouseClick(int button, int state, int x, int y);
			static void glutTrajectoryViewMouseMove(int x, int y);
			static void glutTrajectoryViewMouseWheel(int button, int dir, int x, int y);

			const Vector2i & getWindowSize(void) const
			{ return mainWinSize; }

			float processedTime;
			int processedFrameNo;
			char *outFolder;
			bool needsRefresh;
			ITMUChar4Image *saveImage;

			void Initialise(int & argc, char** argv, ImageSourceEngine *imageSource, IMUSourceEngine *imuSource, ITMMainEngine *mainEngine,
				const char *outFolder, ITMLibSettings::DeviceType deviceType);
			void Shutdown();

			void Run();
			void ProcessFrame();
			
			void GetScreenshot(ITMUChar4Image *dest) const;
			void GetFusionScreenshot(ITMUChar4Image *dest) const;
			void SaveScreenshot(const char *filename) const;
			void SaveFusionScreenshot(const char *filename) const;
			void SaveSceneToMesh(const char *filename) const;
		};
	}
}
