// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "UIEngine.h"
#include <iostream>
#include <strstream>

#include <string.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#ifdef FREEGLUT
#include <GL/freeglut.h>
#else
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "glut64")
#endif
#endif

#include "../Utils/FileUtils.h"

using namespace InfiniTAM::Engine;
UIEngine* UIEngine::instance;

static void safe_glutBitmapString(void *font, const char *str)
{
	size_t len = strlen(str);
	for (size_t x = 0; x < len; ++x) {
		glutBitmapCharacter(font, str[x]);
	}
}

void UIEngine::glutMainWinDisplayFunction(){

}

void UIEngine::glutDisplayFunction()
{
	UIEngine *uiEngine = UIEngine::Instance();

	glutSetWindow(uiEngine->subwin[0]);

	// get updated images from processing thread
	uiEngine->mainEngine->GetImage(uiEngine->outImage[0], uiEngine->outImageType[0], &uiEngine->freeviewPose, &uiEngine->freeviewIntrinsics);

	for (int w = 1; w < NUM_WIN; w++) uiEngine->mainEngine->GetImage(uiEngine->outImage[w], uiEngine->outImageType[w]);

	// do the actual drawing
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0f, 1.0f, 1.0f);
	glEnable(GL_TEXTURE_2D);

	ITMUChar4Image** showImgs = uiEngine->outImage;
	Vector4f *winReg = uiEngine->winReg;
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	{
		glLoadIdentity();
		glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		{
			glEnable(GL_TEXTURE_2D);
			for (int w = 0; w < NUM_WIN; w++)	{// Draw each sub window
				if (uiEngine->outImageType[w] == ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN) continue;
				glBindTexture(GL_TEXTURE_2D, uiEngine->textureId[w]);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, showImgs[w]->noDims.x, showImgs[w]->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, showImgs[w]->GetData(MEMORYDEVICE_CPU));
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glBegin(GL_QUADS); {
					glTexCoord2f(0, 1); glVertex2f(winReg[w][0], winReg[w][1]); // glVertex2f(0, 0);
					glTexCoord2f(1, 1); glVertex2f(winReg[w][2], winReg[w][1]); // glVertex2f(1, 0);
					glTexCoord2f(1, 0); glVertex2f(winReg[w][2], winReg[w][3]); // glVertex2f(1, 1);
					glTexCoord2f(0, 0); glVertex2f(winReg[w][0], winReg[w][3]); // glVertex2f(0, 1);
				}
				glEnd();
			}
			glDisable(GL_TEXTURE_2D);
		}
		glPopMatrix();
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glColor3f(1.0f, 0.0f, 0.0f); glRasterPos2f(0.85f, -0.962f);

	char str[200]; sprintf(str, "%04.2lf", uiEngine->processedTime);
	safe_glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const char*)str);

	glRasterPos2f(-0.95f, -0.95f);
	if (uiEngine->freeviewActive)
	{
		sprintf(str, "n - next frame \t b - all frames \t e/esc - exit \t f - follow camera \t c - colours (currently %s) \t t - turn fusion %s", uiEngine->colourModes[uiEngine->currentColourMode].name, uiEngine->intergrationActive ? "off" : "on");
	}
	else
	{
		sprintf(str, "n - next frame \t b - all frames \t e/esc - exit \t f - free viewpoint \t t - turn fusion %s", uiEngine->intergrationActive ? "off" : "on");
	}
	safe_glutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*)str);

	glutSwapBuffers();
	uiEngine->needsRefresh = false;
}

void UIEngine::glutDisplayTrajectoryFunction()
{
	UIEngine *uiEngine = UIEngine::Instance();

	glutSetWindow(uiEngine->subwin[1]);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (uiEngine->thetaY < 0){
		uiEngine->thetaY = uiEngine->thetaY + 360;
	}
	if (uiEngine->thetaY > 360){
		uiEngine->thetaY = uiEngine->thetaY - 360;
	}
	if (uiEngine->thetaX < 0){
		uiEngine->thetaX = uiEngine->thetaX + 360;
	}
	if (uiEngine->thetaX > 360){
		uiEngine->thetaX = uiEngine->thetaX - 360;
	}

	if (uiEngine->scaleFactor<0){
		uiEngine->scaleFactor = 0;
	}

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glRotatef(uiEngine->thetaY, 0, 1, 0);
	glRotatef(uiEngine->thetaX, 1, 0, 0);
	glScalef(uiEngine->scaleFactor, uiEngine->scaleFactor, uiEngine->scaleFactor);
	glTranslatef(-uiEngine->centerX, -uiEngine->centerY, -uiEngine->centerZ);
	
	gluLookAt(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f);

	glPointSize(2.0f);
	glBegin(GL_POINTS);
	for (int i = 0; i < uiEngine->mainEngine->cpoints_vec.size(); i++){
		std::vector<Vector3f> cpoints = uiEngine->mainEngine->cpoints_vec[i];
		Vector3f color = uiEngine->mainEngine->color_vec[i];

		glColor3f(color[0], color[1], color[2]);
		for (int j = 0; j < cpoints.size(); j++){
			glVertex3f(cpoints[j].x, cpoints[j].y, cpoints[j].z);
		}
	}

	glEnd();

	glLineWidth(1.0);
	for (int i = 0; i < (int)(uiEngine->mainEngine->cpoints_vec.size()) - 1; i++){
		std::vector<Vector3f> cpoints1 = uiEngine->mainEngine->cpoints_vec[i];
        std::vector<Vector3f> cpoints2 = uiEngine->mainEngine->cpoints_vec[i+1];
		Vector3f color = uiEngine->mainEngine->color_vec[i];

		glColor3f(color[0], color[1], color[2]);
		for (int j = 0; j < cpoints1.size(); j++){
			glBegin(GL_LINES);
			glVertex3f(cpoints1[j].x, cpoints1[j].y, cpoints1[j].z);
			glVertex3f(cpoints2[j].x, cpoints2[j].y, cpoints2[j].z);
			glEnd();
		}
	}

	//draw coordinate system
	glLineWidth(6.0);

	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glEnd();

	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);
	glEnd();

	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glEnd();

	glPopMatrix();

	glutSwapBuffers();

	uiEngine->needsRefresh = false;
}

//reshape
void UIEngine::glutReshapeTrajectoryView(int width, int height)
{
	UIEngine *uiEngine = UIEngine::Instance();

	glutSetWindow(uiEngine->subwin[1]);

	glViewport(0, 0, (GLsizei)width, (GLsizei)height);
	/*if (width <= height)
		glViewport(0, 0, (GLsizei)width, (GLsizei)width);
		else
		glViewport(0, 0, (GLsizei)height, (GLsizei)height);*/

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	if (width <= height)
		glOrtho(-8, 8, -8 * (GLfloat)height / (GLfloat)width, 8 * (GLfloat)height / (GLfloat)width, -100.0, 100.0);
	else
		glOrtho(-8 * (GLfloat)width / (GLfloat)height, 8 * (GLfloat)width / (GLfloat)height, -8, 8, -100.0, 100.0);
	//glOrtho(-8, 8, -8, 8, -28, 28);
}

void UIEngine::glutIdleFunction()
{
	UIEngine *uiEngine = UIEngine::Instance();

	switch (uiEngine->mainLoopAction)
	{
	case PROCESS_FRAME:
		uiEngine->ProcessFrame(); uiEngine->processedFrameNo++;
		uiEngine->mainLoopAction = PROCESS_PAUSED;
		uiEngine->needsRefresh = true;
		break;
	case PROCESS_VIDEO:
		uiEngine->ProcessFrame(); uiEngine->processedFrameNo++;
		uiEngine->needsRefresh = true;
		break;
		//case SAVE_TO_DISK:
		//	if (!uiEngine->actionDone)
		//	{
		//		char outFile[255];

		//		ITMUChar4Image *saveImage = uiEngine->saveImage;

		//		glReadBuffer(GL_BACK);
		//		glReadPixels(0, 0, saveImage->noDims.x, saveImage->noDims.x, GL_RGBA, GL_UNSIGNED_BYTE, (unsigned char*)saveImage->GetData(false));
		//		sprintf(outFile, "%s/out_%05d.ppm", uiEngine->outFolder, uiEngine->processedFrameNo);

		//		SaveImageToFile(saveImage, outFile, true);

		//		uiEngine->actionDone = true;
		//	}
		//	break;
	case EXIT:
#ifdef FREEGLUT
		glutLeaveMainLoop();
#else
		exit(0);
#endif
		break;
	case PROCESS_PAUSED:
	default:
		break;
	}

	if (uiEngine->needsRefresh) {
		//glutPostRedisplay();
		uiEngine->glutDisplayFunction();
		uiEngine->glutDisplayTrajectoryFunction();
	}
}

void UIEngine::glutKeyUpFunction(unsigned char key, int x, int y)
{
	UIEngine *uiEngine = UIEngine::Instance();

	switch (key)
	{
	case 'n':
		printf("processing one frame ...\n");
		uiEngine->mainLoopAction = UIEngine::PROCESS_FRAME;
		break;
	case 'b':
		printf("processing input source ...\n");
		uiEngine->mainLoopAction = UIEngine::PROCESS_VIDEO;
		break;
	case 's':
		printf("saving surface points to disk ...");
		uiEngine->mainLoopAction = UIEngine::PROCESS_PAUSED;
		uiEngine->mainEngine->saveSurfacePoints("surfacePoints.ply");
		printf(" done\n");
		break;
	case 'e':
	case 27: // esc key
		printf("exiting ...\n");
		uiEngine->mainLoopAction = UIEngine::EXIT;
		break;
	case 'f':
		if (uiEngine->freeviewActive)
		{
			uiEngine->outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
			uiEngine->outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;

			uiEngine->freeviewActive = false;
		}
		else
		{
			uiEngine->outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED;
			uiEngine->outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;

			uiEngine->freeviewPose.SetFrom(uiEngine->mainEngine->GetTrackingState()->pose_d);
			if (uiEngine->mainEngine->GetView() != NULL) {
				uiEngine->freeviewIntrinsics = uiEngine->mainEngine->GetView()->calib->intrinsics_d;
				uiEngine->outImage[0]->ChangeDims(uiEngine->mainEngine->GetView()->depth->noDims);
			}
			uiEngine->freeviewActive = true;
		}
		uiEngine->needsRefresh = true;
		break;
	case 'c':
		uiEngine->currentColourMode++; if ((unsigned)uiEngine->currentColourMode >= uiEngine->colourModes.size()) uiEngine->currentColourMode = 0;
		uiEngine->needsRefresh = true;
		break;
	case 't':
		uiEngine->intergrationActive = !uiEngine->intergrationActive;
		if (uiEngine->intergrationActive) uiEngine->mainEngine->turnOnIntegration();
		else uiEngine->mainEngine->turnOffIntegration();
		break;
	case 'w':
		printf("saving mesh to disk ...");
		uiEngine->SaveSceneToMesh("mesh.stl");
		printf(" done\n");
		break;
	default:
		break;
	}

	if (uiEngine->freeviewActive) {
		uiEngine->outImageType[0] = uiEngine->colourModes[uiEngine->currentColourMode].type;
	}
}

void UIEngine::glutMouseButtonFunction(int button, int state, int x, int y)
{
	UIEngine *uiEngine = UIEngine::Instance();

	glutSetWindow(uiEngine->subwin[0]);

	if (state == GLUT_DOWN)
	{
		switch (button)
		{
		case GLUT_LEFT_BUTTON: uiEngine->mouseState = 1; break;
		case GLUT_MIDDLE_BUTTON: uiEngine->mouseState = 3; break;
		case GLUT_RIGHT_BUTTON: uiEngine->mouseState = 2; break;
		default: break;
		}
		uiEngine->mouseLastClick.x = x;
		uiEngine->mouseLastClick.y = y;
	}
	else if (state == GLUT_UP) uiEngine->mouseState = 0;
}

static inline Matrix3f createRotation(const Vector3f & _axis, float angle)
{
	Vector3f axis = normalize(_axis);
	float si = sinf(angle);
	float co = cosf(angle);

	Matrix3f ret;
	ret.setIdentity();

	ret *= co;
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) ret.at(c, r) += (1.0f - co) * axis[c] * axis[r];

	Matrix3f skewmat;
	skewmat.setZeros();
	skewmat.at(1, 0) = -axis.z;
	skewmat.at(0, 1) = axis.z;
	skewmat.at(2, 0) = axis.y;
	skewmat.at(0, 2) = -axis.y;
	skewmat.at(2, 1) = axis.x;
	skewmat.at(1, 2) = -axis.x;
	skewmat *= si;
	ret += skewmat;

	return ret;
}

void UIEngine::glutMouseMoveFunction(int x, int y)
{
	UIEngine *uiEngine = UIEngine::Instance();

	glutSetWindow(uiEngine->subwin[0]);

	if (!uiEngine->freeviewActive) return;

	Vector2i movement;
	movement.x = x - uiEngine->mouseLastClick.x;
	movement.y = y - uiEngine->mouseLastClick.y;
	uiEngine->mouseLastClick.x = x;
	uiEngine->mouseLastClick.y = y;

	if ((movement.x == 0) && (movement.y == 0)) return;

	static const float scale_rotation = 0.005f;
	static const float scale_translation = 0.0025f;

	switch (uiEngine->mouseState)
	{
	case 1:
	{
		// left button: rotation
		Vector3f axis((float)-movement.y, (float)-movement.x, 0.0f);
		float angle = scale_rotation * sqrt((float)(movement.x * movement.x + movement.y*movement.y));
		Matrix3f rot = createRotation(axis, angle);
		uiEngine->freeviewPose.SetRT(rot * uiEngine->freeviewPose.GetR(), rot * uiEngine->freeviewPose.GetT());
		uiEngine->freeviewPose.Coerce();
		uiEngine->needsRefresh = true;
		break;
	}
	case 2:
	{
		// right button: translation in x and y direction
		uiEngine->freeviewPose.SetT(uiEngine->freeviewPose.GetT() + scale_translation * Vector3f((float)movement.x, (float)movement.y, 0.0f));
		uiEngine->needsRefresh = true;
		break;
	}
	case 3:
	{
		// middle button: translation along z axis
		uiEngine->freeviewPose.SetT(uiEngine->freeviewPose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, (float)movement.y));
		uiEngine->needsRefresh = true;
		break;
	}
	default: break;
	}
}

void UIEngine::glutMouseWheelFunction(int button, int dir, int x, int y)
{
	UIEngine *uiEngine = UIEngine::Instance();

	glutSetWindow(uiEngine->subwin[0]);

	static const float scale_translation = 0.05f;

	uiEngine->freeviewPose.SetT(uiEngine->freeviewPose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, (dir > 0) ? -1.0f : 1.0f));
	uiEngine->needsRefresh = true;
}

//do when mouse clicks
void UIEngine::glutTrajectoryViewMouseClick(int button, int state, int x, int y)
{
	UIEngine *uiEngine = UIEngine::Instance();
	glutSetWindow(uiEngine->subwin[1]);

	if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON){
		uiEngine->isRotating = true;
		uiEngine->oldx = x, uiEngine->oldy = y;
	}
	if (state == GLUT_UP && button == GLUT_LEFT_BUTTON){
		uiEngine->isRotating = false;
	}
	if (state == GLUT_DOWN && button == GLUT_RIGHT_BUTTON)
	{
		uiEngine->isTranslating = true;
		uiEngine->oldx = x, uiEngine->oldy = y;
	}
	if (state == GLUT_UP && button == GLUT_RIGHT_BUTTON)
	{
		uiEngine->isTranslating = false;
	}
	if (state == GLUT_DOWN && button == GLUT_MIDDLE_BUTTON)
	{
		uiEngine->isScaling = true;
		uiEngine->oldx = x, uiEngine->oldy = y;
	}
	if (state == GLUT_UP && button == GLUT_MIDDLE_BUTTON)
	{
		uiEngine->isScaling = false;
	}
}

void UIEngine::glutTrajectoryViewMouseWheel(int button, int dir, int x, int y)
{
	UIEngine *uiEngine = UIEngine::Instance();
	glutSetWindow(uiEngine->subwin[1]);

	if (dir == 1)
	{
		uiEngine->scaleFactor += 0.03;
		glutPostRedisplay();
	}
	if (dir == -1)
	{
		uiEngine->scaleFactor -= 0.03;
		glutPostRedisplay();
	}
}

//do when mouse moves
void UIEngine::glutTrajectoryViewMouseMove(int x, int y)
{
	UIEngine *uiEngine = UIEngine::Instance();
	glutSetWindow(uiEngine->subwin[1]);
	
	if (uiEngine->isRotating){
		uiEngine->dx += (x - uiEngine->oldx);
		uiEngine->dy += (y - uiEngine->oldy);
		uiEngine->thetaX = uiEngine->dy / uiEngine->subWin2Size.y * 90;
		uiEngine->thetaY = uiEngine->dx / uiEngine->subWin2Size.x * 90;
		uiEngine->oldx = x, uiEngine->oldy = y;
		glutPostRedisplay();
	}
	if (uiEngine->isTranslating){
		uiEngine->centerX -= ((x - uiEngine->oldx) / uiEngine->subWin2Size.x);
		uiEngine->centerY += ((y - uiEngine->oldy) / uiEngine->subWin2Size.y);
		uiEngine->oldx = x, uiEngine->oldy = y;
		glutPostRedisplay();
	}
	if (uiEngine->isScaling){
		if ((y - uiEngine->oldy) < 0){
			uiEngine->scaleFactor += 0.2f;
		}
		else if ((y - uiEngine->oldy) > 0){
			uiEngine->scaleFactor -= 0.2f;
		}

		uiEngine->oldx = x, uiEngine->oldy = y;
		glutPostRedisplay();
	}
}

void UIEngine::Initialise(int & argc, char** argv, ImageSourceEngine *imageSource, IMUSourceEngine *imuSource, ITMMainEngine *mainEngine,
	const char *outFolder, ITMLibSettings::DeviceType deviceType)
{
	thetaX = 0.0;
	thetaY = 0.0;
	scaleFactor = 1.0;

	dx = 0;
	dy = 0;
	oldy = -1;
	oldx = -1;

	centerX = 0;
	centerY = 0;
	centerZ = 0;

	isRotating = false;
	isTranslating = false;
	isScaling = false;

	this->freeviewActive = false;
	this->intergrationActive = true;
	this->currentColourMode = 0;
	this->colourModes.push_back(UIColourMode("shaded greyscale", ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED));
	if (ITMVoxel::hasColorInformation) this->colourModes.push_back(UIColourMode("integrated colours", ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME));
	this->colourModes.push_back(UIColourMode("surface normals", ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL));

	this->imageSource = imageSource;
	this->imuSource = imuSource;
	this->mainEngine = mainEngine;
	{
		size_t len = strlen(outFolder);
		this->outFolder = new char[len + 1];
		strcpy(this->outFolder, outFolder);
	}

	//Vector2i winSize;
	//int textHeight = 30; // Height of text area
	//winSize.x = 2 * MAX(imageSource->getRGBImageSize().x, imageSource->getDepthImageSize().x);
	//winSize.y = MAX(imageSource->getRGBImageSize().y, imageSource->getDepthImageSize().y) + textHeight;
	//float h1 = textHeight / (float)winSize.y, h2 = (1.f + h1) / 2;
	//winReg[0] = Vector4f(0, h1, 0.5, 1); // Main render
	//winReg[1] = Vector4f(0.5, h2, 0.75, 1); // Side sub window 0
	//winReg[2] = Vector4f(0.75, h2, 1, 1); // Side sub window 1
	//winReg[3] = Vector4f(0.5, h1, 0.75, h2); // Side sub window 2
	//winReg[4] = Vector4f(0.75, h1, 1, h2); // Side sub window 3

	int textHeight = 30; // Height of text area
	//winSize.x = (int)(1.5f * (float)MAX(imageSource->getImageSize().x, imageSource->getDepthImageSize().x));
	//winSize.y = MAX(imageSource->getRGBImageSize().y, imageSource->getDepthImageSize().y) + textHeight;
	subWin1Size.x = (int)(1.5f * (float)(imageSource->getDepthImageSize().x));
	subWin1Size.y = imageSource->getDepthImageSize().y + textHeight;
	float h1 = textHeight / (float)subWin1Size.y, h2 = (1.f + h1) / 2;
	winReg[0] = Vector4f(0.0f, h1, 0.665f, 1.0f);   // Main render
	winReg[1] = Vector4f(0.665f, h2, 1.0f, 1.0f);   // Side sub window 0
	winReg[2] = Vector4f(0.665f, h1, 1.0f, h2);     // Side sub window 2

	subWin2Size = subWin1Size;
	mainWinSize.x = subWin1Size.x;
	mainWinSize.y = subWin1Size.y + subWin2Size.y;

	this->isRecording = false;
	this->currentFrameNo = 0;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(mainWinSize.x, mainWinSize.y);
	int mainWindow = glutCreateWindow("InfiniTAM");
	glutDisplayFunc(UIEngine::glutMainWinDisplayFunction);

	glGenTextures(NUM_WIN, textureId);
	glutKeyboardUpFunc(UIEngine::glutKeyUpFunction);
	glutIdleFunc(UIEngine::glutIdleFunction);

	subwin[0] = glutCreateSubWindow(mainWindow, 0, 0, subWin1Size.x, subWin1Size.y);
	glutDisplayFunc(UIEngine::glutDisplayFunction);
	glutMouseFunc(UIEngine::glutMouseButtonFunction);
	glutMotionFunc(UIEngine::glutMouseMoveFunction);

#ifdef FREEGLUT
	glutMouseWheelFunc(UIEngine::glutMouseWheelFunction);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, 1);
#endif

	subwin[1] = glutCreateSubWindow(mainWindow, 0, subWin1Size.y, subWin2Size.x, subWin2Size.y);
	glutDisplayFunc(UIEngine::glutDisplayTrajectoryFunction);
	glutReshapeFunc(UIEngine::glutReshapeTrajectoryView);
	//glutMouseWheelFunc(UIEngine::glutTrajectoryViewMouseWheel);
	glutMouseFunc(UIEngine::glutTrajectoryViewMouseClick);
	glutMotionFunc(UIEngine::glutTrajectoryViewMouseMove);

	bool allocateGPU = false;
	if (deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

	for (int w = 0; w < NUM_WIN; w++)
		outImage[w] = new ITMUChar4Image(imageSource->getDepthImageSize(), true, allocateGPU);

	inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new ITMIMUMeasurement();

	saveImage = new ITMUChar4Image(imageSource->getDepthImageSize(), true, false);

	outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
	outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
	outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
	if (inputRGBImage->noDims == Vector2i(0, 0)) outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN;
	//outImageType[3] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
	//outImageType[4] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;

	mainLoopAction = PROCESS_PAUSED;
	mouseState = 0;
	needsRefresh = false;
	processedFrameNo = 0;
	processedTime = 0.0f;

#ifndef COMPILE_WITHOUT_CUDA
	ITMSafeCall(cudaThreadSynchronize());
#endif

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);

	sdkResetTimer(&timer_average);

	printf("initialised.\n");
}

void UIEngine::SaveScreenshot(const char *filename) const
{
	ITMUChar4Image screenshot(getWindowSize(), true, false);
	GetScreenshot(&screenshot);
	SaveImageToFile(&screenshot, filename, true);
}

void UIEngine::SaveFusionScreenshot(const char *filename) const
{
	UIEngine *uiEngine = UIEngine::Instance();
	Vector4f *winReg = uiEngine->winReg;
	Vector2i window;
	window.x = (int)((winReg[0][2] - winReg[0][0]) * subWin1Size.x);
	window.y = (int)((winReg[0][3] - winReg[0][1]) * subWin1Size.y);

	ITMUChar4Image screenshot(window, true, false);
	GetFusionScreenshot(&screenshot);
	SaveImageToFile(&screenshot, filename, true);
}

void UIEngine::SaveSceneToMesh(const char *filename) const
{
	mainEngine->SaveSceneToMesh(filename);
}

void UIEngine::GetScreenshot(ITMUChar4Image *dest) const
{
	glReadPixels(0, 0, dest->noDims.x, dest->noDims.y, GL_RGBA, GL_UNSIGNED_BYTE, dest->GetData(MEMORYDEVICE_CPU));
}

void UIEngine::GetFusionScreenshot(ITMUChar4Image *dest) const
{
	int textHeight = 30;
	glReadPixels(0, textHeight, dest->noDims.x, dest->noDims.y + textHeight, GL_RGBA, GL_UNSIGNED_BYTE, dest->GetData(MEMORYDEVICE_CPU));
}


void UIEngine::ProcessFrame()
{
	std::cout << "process frame "<< currentFrameNo << std::endl;

    //just for debug
	UIEngine *uiEngine = UIEngine::Instance();
	if (currentFrameNo == 11){
		uiEngine->mainLoopAction = UIEngine::PROCESS_FRAME;
	}

	if (!imageSource->hasMoreImages()) return;
	imageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (imuSource != NULL) {
		if (!imuSource->hasMoreMeasurements()) return;
		else imuSource->getMeasurement(inputIMUMeasurement);
	}

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant); sdkStartTimer(&timer_average);

	//actual processing on the mailEngine
	if (imuSource != NULL) mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	else mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

#ifndef COMPILE_WITHOUT_CUDA
	ITMSafeCall(cudaThreadSynchronize());
#endif
	sdkStopTimer(&timer_instant); sdkStopTimer(&timer_average);

	//processedTime = sdkGetTimerValue(&timer_instant);
	processedTime = sdkGetAverageTimerValue(&timer_average);

	if (isRecording){
		std::strstream ss1;
		std::string snapshotname1;
		ss1 << "snapshots/full_" << currentFrameNo << ".ppm";
		ss1 >> snapshotname1;
		SaveScreenshot(snapshotname1.c_str());

		std::strstream ss2;
		std::string snapshotname2;
		ss2 << "snapshots/fusion_" << currentFrameNo << ".ppm";
		ss2 >> snapshotname2;
		SaveFusionScreenshot(snapshotname2.c_str());
	}

	currentFrameNo++;
}

void UIEngine::Run() { glutMainLoop(); }
void UIEngine::Shutdown()
{
	sdkDeleteTimer(&timer_instant);
	sdkDeleteTimer(&timer_average);

	for (int w = 0; w < NUM_WIN; w++)
		delete outImage[w];

	delete inputRGBImage;
	delete inputRawDepthImage;
	delete inputIMUMeasurement;

	delete[] outFolder;
	delete saveImage;
	delete instance;
}