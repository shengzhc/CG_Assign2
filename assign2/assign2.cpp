// assign2.cpp : Defines the entry point for the console application.
//

/*
	CSCI 480 Computer Graphics
	Assignment 2: Simulating a Roller Coaster
	C++ starter code
*/

#include "stdafx.h"
#include <pic.h>
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <GL/glu.h>
#include <GL/glut.h>

#define KEY_S 0x73
#define KEY_L 0x6c
#define KEY_C 0x63
#define KEY_A 0x61


#define FPS 15
#define TPF ((long int)(1000.0/15 * 0.75))
#define MAX_SCREENSHOTS 1000

// Camera FOV
#define FOV 90.0f

// Basic control matrix factor
#define S 0.5f

// slices each 0-1.0 when drawing spline segment
#define U 1000

// terrain span
#define TERRAIN_Z 10
#define TERRAIN_X 10
#define TERRAIN_Y -0.5f

// skybox size
#define SKYBOX_SIZE 25.0f
#define SKYBOX_STRIP 1.0f

// define a strip for paddles
#define PADDLE_STRIP 30

typedef enum { ROTATE, TRANSLATE, SCALE } CONTROLSTATE;
CONTROLSTATE g_ControlState = ROTATE;

// screenshot destination folder
char *folderName = "screenshot/";

// terrain texture
const char *terrainPic = "stonewall.jpg";
GLuint terrainTex_id;

// camera moving speed
int cam_movingSpeed = 25;

// skybox texture
const char *skyboxBK = "violent_BK.jpg";
GLuint skyboxTexBK_id;
const char *skyboxDN = "violent_DN.jpg";
GLuint skyboxTexDN_id;
const char *skyboxFR = "violent_FR.jpg";
GLuint skyboxTexFR_id;
const char *skyboxLF = "violent_LF.jpg";
GLuint skyboxTexLF_id;
const char *skyboxRT = "violent_RT.jpg";
GLuint skyboxTexRT_id;
const char *skyboxUP = "violent_UP.jpg";
GLuint skyboxTexUP_id;

// paddler texture
const char *paddlerFile = "paddle_texture.jpg";
GLuint paddlerTex_id;
/* represents one control point along the spline */
struct point {
	double x;
	double y;
	double z;
	double tangent[3];
	double normal[3];
	double binormal[3];
};

// define a camera properties
struct camera{
	double eye[3];
	double lookat[3];
	double up[3];
};

/* spline struct which contains how many control points, and an array of control points */
struct spline {
	int numControlPoints;
	struct point *points;
};

// define a segmet. A spline consists of several segments.
struct segment
{
	int c_points;
	double matControl[4][3];
	point *pointsPtr;
};

/* the spline array */
struct spline *g_Splines;
struct camera cam;

/* total number of splines */
int g_iNumOfSplines;

// current camera position
double g_iTimeStep = 0.0;

float g_vLandRotate[3] = {0.0, 0.0, 0.0};
float g_vLandTranslate[3] = {0.0, 0.0, 0.0};
float g_vLandScale[3] = {1.0, 1.0, 1.0};
int g_vMousePos[2] = {0, 0};
int g_iLeftMouseButton = 0;    /* 1 if pressed, 0 if not */
int g_iMiddleMouseButton = 0;
int g_iRightMouseButton = 0;

// store all the segments data (pointer)
segment *splineSegments;

// Switches that handle certain features
bool isLightOn = true;
bool isStartingAnimation = false;
bool isCameraMovable = true;
GLfloat basicMatrix[4][4] = {{-S, 2-S, S-2, S}, {2*S, S-3, 3-2*S, -S}, {-S, 0, S, 0}, {0, 1, 0, 0}};
 
int loadSplines(char *argv);
void crossproduct(double dest[3], double vert1[3], double vert2[3]);
void normalistVertor(double vert[3]);
void interpolateVertex(point *vertex, double controlMat[4][3], double u);
void interpolateVertexTangent(point *vertex, double controlMat[4][3], double u);
void setupTerrain();
void setupSkybox();
void setupLight();
void setupPaddle();
void initSegment();
void init();
void doIdle();
void drawTerrain();
void drawSkybox();
void drawRollCoaster();
void drawPaddlers();
void display();
void reshape(int w, int h);
void mousedrag(int x, int y);
void mouseidle(int x, int y);
void mousebutton(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);
void getFileName(int id, char filename[]);
void scheduleForScreenShot();
void saveScreenshot(char *filename);

// Load splines from file
int loadSplines(char *argv) {
	char *cName = (char *)malloc(128 * sizeof(char));
	FILE *fileList;
	FILE *fileSpline;
	int iType, i = 0, j, iLength;

	/* load the track file */
	fileList = fopen(argv, "r");
	if (fileList == NULL) {
		printf ("can't open file\n");
		exit(1);
	}
  
	/* stores the number of splines in a global variable */
	fscanf(fileList, "%d", &g_iNumOfSplines);

	g_Splines = (struct spline *)malloc(g_iNumOfSplines * sizeof(struct spline));

	/* reads through the spline files */
	for (j = 0; j < g_iNumOfSplines; j++) {
		i = 0;
		fscanf(fileList, "%s", cName);
		fileSpline = fopen(cName, "r");

		if (fileSpline == NULL) {
			printf ("can't open file\n");
			exit(1);
		}

		/* gets length for spline file */
		fscanf(fileSpline, "%d %d", &iLength, &iType);

		/* allocate memory for all the points */
		g_Splines[j].points = (struct point *)malloc(iLength * sizeof(struct point));
		g_Splines[j].numControlPoints = iLength;

		/* saves the data to the struct */
		while (fscanf(fileSpline, "%lf %lf %lf", 
			&g_Splines[j].points[i].x, 
			&g_Splines[j].points[i].y, 
			&g_Splines[j].points[i].z) != EOF) {
			i++;
		}
	}

	free(cName);

	return 0;
}

// Do cross product operation
void crossproduct(double dest[3], double vert1[3], double vert2[3])
{
	dest[0] = vert1[1]*vert2[2] - vert1[2]*vert2[1];
	dest[1] = vert1[2]*vert2[0] - vert1[0]*vert2[2];
	dest[2] = vert1[0]*vert2[1] - vert1[1]*vert2[0];
	normalistVertor(dest);
}
// normalize a vertor
void normalistVertor(double vert[3])
{
	double length = sqrt(pow(vert[0], 2.0)+pow(vert[1], 2.0)+pow(vert[2], 2.0));
	if (length != 0)
	{	
		for (int i=0; i<3; i++)
		{
			vert[i]/=length;
		}
	}
}
// interpolate a vertex in world coordinate with a given u and current CM
void interpolateVertex(point *vertex, double controlMat[4][3], double u)
{
	double temp[4];
	double uArr[4];
	for (int i=3; i>=0; i--)
	{
		uArr[i] = pow(u, 3-i);
	}
	for (int i=0; i<4; i++)
	{
		double sum = 0.0;
		for (int j=0; j<4; j++)
		{
			sum += uArr[j]*basicMatrix[j][i];
		}
		temp[i] = sum;
	}
	vertex->x = 0.0;
	vertex->y = 0.0;
	vertex->z = 0.0;
	for (int j=0; j<4; j++)
	{
		vertex->x += temp[j] * controlMat[j][0];
		vertex->y += temp[j] * controlMat[j][1];
		vertex->z += temp[j] * controlMat[j][2];
	}
}
// interpolate a vertex tangent in world coordinate
void interpolateVertexTangent(point *vertex, double controlMat[4][3], double u)
{
	double temp[4];
	double uArr[4];
	uArr[0] = pow(u, 2.0)*3;
	uArr[1] = pow(u, 1.0)*2;
	uArr[2] = pow(u, 0.0)*1;
	uArr[3] = 0.0;

	for (int i=0; i<4; i++)
	{
		double sum = 0.0;
		for (int j=0; j<4; j++)
		{
			sum += uArr[j]*basicMatrix[j][i];
		}
		temp[i] = sum;
	}
	vertex->tangent[0] = 0.0;
	vertex->tangent[1] = 0.0;
	vertex->tangent[2] = 0.0;
	for (int j=0; j<4; j++)
	{
		vertex->tangent[0] += temp[j] * controlMat[j][0];
		vertex->tangent[1] += temp[j] * controlMat[j][1];
		vertex->tangent[2] += temp[j] * controlMat[j][2];
	}
	normalistVertor(vertex->tangent);
}
// the belowing setup*s are used to load specific files and features, and set up the environment
void setupTerrain()
{
	Pic *terrainTex = jpeg_read((char *)terrainPic, NULL);
	glGenTextures(1, &terrainTex_id);
	glBindTexture(GL_TEXTURE_2D, terrainTex_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	switch(terrainTex->bpp)
	{
	case 1:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, terrainTex->nx, terrainTex->ny, 0, GL_RGB8, GL_UNSIGNED_BYTE, terrainTex->pix);
		break;
	case 3:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, terrainTex->nx, terrainTex->ny, 0, GL_RGB, GL_UNSIGNED_BYTE, terrainTex->pix);
		break;
	case 4:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, terrainTex->nx, terrainTex->ny, 0, GL_RGBA, GL_UNSIGNED_BYTE, terrainTex->pix);
		break;
	default:
		break;
	}
}
void setupSkybox()
{
	Pic *sbBK = jpeg_read((char *)skyboxBK, NULL);
	glGenTextures(1, &skyboxTexBK_id);
	glBindTexture(GL_TEXTURE_2D, skyboxTexBK_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	switch(sbBK->bpp)
	{
	case 1:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, sbBK->nx, sbBK->ny, 0, GL_RGB8, GL_UNSIGNED_BYTE, sbBK->pix);
		break;
	case 3:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, sbBK->nx, sbBK->ny, 0, GL_RGB, GL_UNSIGNED_BYTE, sbBK->pix);
		break;
	case 4:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sbBK->nx, sbBK->ny, 0, GL_RGBA, GL_UNSIGNED_BYTE, sbBK->pix);
		break;
	default:
		break;
	}

	Pic *sbDN = jpeg_read((char *)skyboxDN, NULL);
	glGenTextures(1, &skyboxTexDN_id);
	glBindTexture(GL_TEXTURE_2D, skyboxTexDN_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	switch(sbDN->bpp)
	{
	case 1:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, sbDN->nx, sbDN->ny, 0, GL_RGB8, GL_UNSIGNED_BYTE, sbDN->pix);
		break;
	case 3:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, sbDN->nx, sbDN->ny, 0, GL_RGB, GL_UNSIGNED_BYTE, sbDN->pix);
		break;
	case 4:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sbDN->nx, sbDN->ny, 0, GL_RGBA, GL_UNSIGNED_BYTE, sbDN->pix);
		break;
	default:
		break;
	}

	Pic *sbFR = jpeg_read((char *)skyboxFR, NULL);
	glGenTextures(1, &skyboxTexFR_id);
	glBindTexture(GL_TEXTURE_2D, skyboxTexFR_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	switch(sbFR->bpp)
	{
	case 1:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, sbFR->nx, sbFR->ny, 0, GL_RGB8, GL_UNSIGNED_BYTE, sbFR->pix);
		break;
	case 3:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, sbFR->nx, sbFR->ny, 0, GL_RGB, GL_UNSIGNED_BYTE, sbFR->pix);
		break;
	case 4:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sbFR->nx, sbFR->ny, 0, GL_RGBA, GL_UNSIGNED_BYTE, sbFR->pix);
		break;
	default:
		break;
	}

	Pic *sbLF = jpeg_read((char *)skyboxLF, NULL);
	glGenTextures(1, &skyboxTexLF_id);
	glBindTexture(GL_TEXTURE_2D, skyboxTexLF_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	switch(sbLF->bpp)
	{
	case 1:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, sbLF->nx, sbLF->ny, 0, GL_RGB8, GL_UNSIGNED_BYTE, sbLF->pix);
		break;
	case 3:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, sbLF->nx, sbLF->ny, 0, GL_RGB, GL_UNSIGNED_BYTE, sbLF->pix);
		break;
	case 4:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sbLF->nx, sbLF->ny, 0, GL_RGBA, GL_UNSIGNED_BYTE, sbLF->pix);
		break;
	default:
		break;
	}

	Pic *sbRT = jpeg_read((char *)skyboxRT, NULL);
	glGenTextures(1, &skyboxTexRT_id);
	glBindTexture(GL_TEXTURE_2D, skyboxTexRT_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	switch(sbRT->bpp)
	{
	case 1:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, sbRT->nx, sbRT->ny, 0, GL_RGB8, GL_UNSIGNED_BYTE, sbRT->pix);
		break;
	case 3:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, sbRT->nx, sbRT->ny, 0, GL_RGB, GL_UNSIGNED_BYTE, sbRT->pix);
		break;
	case 4:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sbRT->nx, sbRT->ny, 0, GL_RGBA, GL_UNSIGNED_BYTE, sbRT->pix);
		break;
	default:
		break;
	}

	Pic *sbUP = jpeg_read((char *)skyboxUP, NULL);
	glGenTextures(1, &skyboxTexUP_id);
	glBindTexture(GL_TEXTURE_2D, skyboxTexUP_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	switch(sbUP->bpp)
	{
	case 1:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, sbUP->nx, sbUP->ny, 0, GL_RGB8, GL_UNSIGNED_BYTE, sbUP->pix);
		break;
	case 3:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, sbUP->nx, sbUP->ny, 0, GL_RGB, GL_UNSIGNED_BYTE, sbUP->pix);
		break;
	case 4:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sbUP->nx, sbUP->ny, 0, GL_RGBA, GL_UNSIGNED_BYTE, sbUP->pix);
		break;
	default:
		break;
	}
}
void setupLight()
{
	GLfloat light0[] = {0.4, 0.4, 0.4, 1.0};
	GLfloat light1[] = {0.3, 0.3, 0.3, 1.0};

	GLfloat light0Pos[] = {1.0, 0.0, 0, 0.0};
	GLfloat light1Pos[] = {0.0, -1.0, 0.0, 0.0};

	// iron optical material property from GOOGLE SEARCH
	GLfloat ambCof[] = {0.0, 0.0, 0.0};
	GLfloat difCof[] = {0.35, 0.35, 0.35};
	GLfloat speCof[] = {0.65, 0.65, 0.65};

	glLightfv(GL_LIGHT0, GL_POSITION, light0Pos);
	glLightfv(GL_LIGHT1, GL_POSITION, light1Pos);

	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0);

	glLightfv(GL_LIGHT1, GL_DIFFUSE, light1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light1);

	// set up material response co-efficients
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambCof);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, difCof);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speCof);

	// front and back normal of a surface should be different 
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
}
void setupPaddle()
{
	Pic *paddleTex = jpeg_read((char *)paddlerFile, NULL);
	glGenTextures(1, &paddlerTex_id);
	glBindTexture(GL_TEXTURE_2D, paddlerTex_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	switch(paddleTex->bpp)
	{
	case 1:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, paddleTex->nx, paddleTex->ny, 0, GL_RGB8, GL_UNSIGNED_BYTE, paddleTex->pix);
		break;
	case 3:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, paddleTex->nx, paddleTex->ny, 0, GL_RGB, GL_UNSIGNED_BYTE, paddleTex->pix);
		break;
	case 4:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, paddleTex->nx, paddleTex->ny, 0, GL_RGBA, GL_UNSIGNED_BYTE, paddleTex->pix);
		break;
	default:
		break;
	}

}
// init and process the segment data
void initSegment()
{
	// pre-calculate each detailed points along the roll coaster
	spline *line = (spline*)(g_Splines+0);
	int numControlPoints = line->numControlPoints;
	splineSegments = (segment *)malloc(sizeof(segment)*(numControlPoints-1));
	double lastBinomal[3] = {1.0, 0.0, 1.0};
	for (int i=0; i<numControlPoints-1; i++)
	{
		segment *seg = (segment *)(splineSegments+i);
		seg->c_points = U;
		if (0==i)
		{
			memcpy(seg->matControl[0], &line->points[0].x, sizeof(double)*3);
			memcpy(seg->matControl[1], &line->points[0].x, sizeof(double)*3);
			memcpy(seg->matControl[2], &line->points[1].x, sizeof(double)*3);
			memcpy(seg->matControl[3], &line->points[2].x, sizeof(double)*3);
		} 
		else if((i+2) == numControlPoints)
		{
			memcpy(seg->matControl[0], &line->points[numControlPoints-3].x, sizeof(double)*3);
			memcpy(seg->matControl[1], &line->points[numControlPoints-2].x, sizeof(double)*3);
			memcpy(seg->matControl[2], &line->points[numControlPoints-1].x, sizeof(double)*3);
			memcpy(seg->matControl[3], &line->points[numControlPoints-1].x, sizeof(double)*3);
		}
		else
		{
			for (int row=0; row<4; row++)
			{
				memcpy(seg->matControl[row], &line->points[i-1+row].x, sizeof(double)*3);
			}
		}
		seg->pointsPtr = (point *)malloc(sizeof(point)*U);
		for (int j=0;j<U; j++)
		{
			interpolateVertex(seg->pointsPtr+j, seg->matControl, j*1.0f/U);
			interpolateVertexTangent(seg->pointsPtr+j, seg->matControl, j*1.0f/U);
			normalistVertor((seg->pointsPtr+j)->tangent);
			crossproduct((seg->pointsPtr+j)->normal, lastBinomal, (seg->pointsPtr+j)->tangent);
			crossproduct((seg->pointsPtr+j)->binormal, (seg->pointsPtr+j)->tangent, (seg->pointsPtr+j)->normal);
			memcpy(lastBinomal, (seg->pointsPtr+j)->binormal, sizeof(double)*3);
		}
	}

}
void init()
{
	glShadeModel(GL_SMOOTH);
	setupTerrain();
	setupSkybox();
	setupPaddle();
	setupLight();
	initSegment();
}
void doIdle()
{
	// moving the camera
	if (isCameraMovable)
	{
		g_iTimeStep += cam_movingSpeed;
	}

	// saving the screenshot
	if (isStartingAnimation)
	{
		scheduleForScreenShot();
	}
	glutPostRedisplay();
}
// the belowing draw*s are used to draw the requirements on scene
void drawTerrain()
{
	/*
		Early implementation. So the drawing sequence is similar to HW1 until found that texture can be 
		automatically interpolated with a rectangle.
	*/

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, terrainTex_id);
	for (float z=-TERRAIN_Z; z<TERRAIN_Z-0.1; z+=0.1)
	{
		for (float  x=-TERRAIN_X; x<TERRAIN_X-0.1; x+=0.1)
		{
			glBegin(GL_TRIANGLES);
			glTexCoord2f((x+TERRAIN_X)/TERRAIN_X/2.0, (z+TERRAIN_Z)/TERRAIN_Z/2.0);
			glVertex3f(x, TERRAIN_Y, z);
			glTexCoord2f((x+0.1+TERRAIN_X)/TERRAIN_X/2.0, (z+TERRAIN_Z)/TERRAIN_Z/2.0);
			glVertex3f(x+0.1, TERRAIN_Y, z);
			glTexCoord2f((x+0.1+TERRAIN_X)/TERRAIN_X/2.0, (z+0.1+TERRAIN_Z)/TERRAIN_Z/2.0);
			glVertex3f(x+0.1, TERRAIN_Y, z+0.1);
			glEnd();
			glBegin(GL_TRIANGLES);
			glTexCoord2f((x+0.1+TERRAIN_X)/TERRAIN_X/2.0, (z+0.1+TERRAIN_Z)/TERRAIN_Z/2.0);
			glVertex3f(x+0.1, TERRAIN_Y, z+0.1);
			glTexCoord2f((x+TERRAIN_X)/TERRAIN_X/2.0, (z+0.1+TERRAIN_Z)/TERRAIN_Z/2.0);
			glVertex3f(x, TERRAIN_Y, z+0.1);
			glTexCoord2f((x+TERRAIN_X)/TERRAIN_X/2.0, (z+TERRAIN_Z)/TERRAIN_Z/2.0);
			glVertex3f(x, TERRAIN_Y, z);
			glEnd();
		}
	}
	glDisable(GL_TEXTURE_2D);

}
void drawSkybox()
{
		/* Descirption
		it is implemented with a skybox. Use 6 pictures to texture each size of a big cube and it looks like that the sky
		is a cube. Seperately loaded the 6 pictures and carefully arrange their positions to make sure sides are continuous.
		*/
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, skyboxTexBK_id);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(-SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(-SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, skyboxTexFR_id);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, skyboxTexUP_id);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, skyboxTexDN_id);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, skyboxTexLF_id);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(-SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(-SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, skyboxTexRT_id);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE);
	glEnd();

	glDisable(GL_TEXTURE_2D);
}
void drawRollCoaster()
{
	/* 
		Draw the two-track road with pre-computed data. It is a little tracky here.
		So we know the few starting points and lines, and make sure that the camera's up vector is world up to easily simulate the whole scene.
		And then I do not draw the computed spline, instead, I add and minus a scalor of binormal vector at each points to make the computed spline as 
		our center line. So that there will be two track around the central axis. And the moving of camera is along the central axis.
	*/
	for (int sLine = 0; sLine<g_iNumOfSplines; sLine++)
	{
		spline *line = g_Splines+sLine;
		glLineWidth(10.0);
		for (int i=0; i<line->numControlPoints-1; i++)
		{
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 0.0);
			segment *segm = (segment *)(splineSegments+i);
			for (int j=0; j<segm->c_points; j++)
			{
				glVertex3f(segm->pointsPtr[j].x+0.2*segm->pointsPtr[j].binormal[0], segm->pointsPtr[j].y+0.2*segm->pointsPtr[j].binormal[1], segm->pointsPtr[j].z+0.2*segm->pointsPtr[j].binormal[2]);
			}
			glEnd();
		}

		for (int i=0; i<line->numControlPoints-1; i++)
		{
			glBegin(GL_LINE_STRIP);
			glColor3f(1.0, 1.0, 0.0);
			segment *segm = (segment *)(splineSegments+i);
			for (int j=0; j<segm->c_points; j++)
			{
				glVertex3f(segm->pointsPtr[j].x-0.2*segm->pointsPtr[j].binormal[0], segm->pointsPtr[j].y-0.2*segm->pointsPtr[j].binormal[1], segm->pointsPtr[j].z-0.2*segm->pointsPtr[j].binormal[2]);
			}
			glEnd();
		}
	}
}
void drawPaddlers()
{
	/* 
		Drawing paddle is similar to draw roll coaster. use a bigger scalor of binormal vector to get 4 points on the track, 
		and then draw a quad with texture.
	*/
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, paddlerTex_id);

	for (int sLine = 0; sLine<g_iNumOfSplines; sLine++)
	{
		spline *line = (spline *)(g_Splines+sLine);
		int numControlPointers = line->numControlPoints;

		for (int i=0; i<numControlPointers-1; i++)
		{
			segment *seg = (segment *)(splineSegments+i);
			for (int j=0; j+PADDLE_STRIP<seg->c_points; j+=(150-PADDLE_STRIP))
			{
				glBegin(GL_QUADS);
				glTexCoord2f(0.0, 0.0);
				glVertex3f(seg->pointsPtr[j].x+seg->pointsPtr[j].binormal[0]*0.3, seg->pointsPtr[j].y+seg->pointsPtr[j].binormal[1]*0.3, seg->pointsPtr[j].z+seg->pointsPtr[j].binormal[2]*0.3);
				glTexCoord2f(1.0, 0.0);
				glVertex3f(seg->pointsPtr[j].x-seg->pointsPtr[j].binormal[0]*0.3, seg->pointsPtr[j].y-seg->pointsPtr[j].binormal[1]*0.3, seg->pointsPtr[j].z-seg->pointsPtr[j].binormal[2]*0.3);
				j += PADDLE_STRIP;
				glTexCoord2f(1.0, 1.0);
				glVertex3f(seg->pointsPtr[j].x-seg->pointsPtr[j].binormal[0]*0.3, seg->pointsPtr[j].y-seg->pointsPtr[j].binormal[1]*0.3, seg->pointsPtr[j].z-seg->pointsPtr[j].binormal[2]*0.3);
				glTexCoord2f(0.0, 1.0);
				glVertex3f(seg->pointsPtr[j].x+seg->pointsPtr[j].binormal[0]*0.3, seg->pointsPtr[j].y+seg->pointsPtr[j].binormal[1]*0.3, seg->pointsPtr[j].z+seg->pointsPtr[j].binormal[2]*0.3);
				glEnd();
			}
		}
	}
	glDisable(GL_TEXTURE_2D);
}
void display()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	// Check the camera is static or dynamic
	if (isCameraMovable)
	{
		int line = (int)g_iTimeStep / U;
		int pos = (int)g_iTimeStep % U;
		if (line > (g_Splines+0)->numControlPoints-2)
		{
			line = 0;
			g_iTimeStep = pos;
			cam_movingSpeed = 20;
		}
		point *p = ((splineSegments+line)->pointsPtr+pos);
		gluLookAt(p->x+p->normal[0]*0.2, p->y+p->normal[1]*0.2, p->z+p->normal[2]*0.2, p->tangent[0]*0.01+p->x+p->normal[0]*0.2, p->tangent[1]*0.01+p->y+p->normal[1]*0.2, p->tangent[2]*0.01+p->z+p->normal[2]*0.2, p->normal[0], p->normal[1], p->normal[2]);

		// simply simulate the physics world to change the speed 
		if (p->tangent[1] > 0 && cam_movingSpeed>7)
			cam_movingSpeed --;
		else if (p->tangent[1] < 0 && cam_movingSpeed<40)
			cam_movingSpeed++;
	}
	else 
	{
		gluLookAt(0.0, 2.5, 10.0, 0.0, 2.5, 0.0, 0.0, 1.0, 0.0);
		glTranslatef(g_vLandTranslate[0], g_vLandTranslate[1], g_vLandTranslate[2]);
		glRotatef(g_vLandRotate[0], 1.0, 0.0, 0.0);
		glRotatef(g_vLandRotate[1], 0.0, 1.0, 0.0);
		glRotatef(g_vLandRotate[2], 0.0, 0.0, 1.0);
		glScalef(g_vLandScale[0], g_vLandScale[1], g_vLandScale[2]);
		cam_movingSpeed = 20;
	}
	
	drawTerrain();
	drawSkybox();
	drawRollCoaster();
	drawPaddlers();
	glutSwapBuffers();
}
void reshape(int w, int h)
{
	GLfloat aspect = w/h;
	// revise projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(FOV, aspect, 0.001f, 1000.0f);
	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, w, h);
}
void mousedrag(int x, int y)
{
	int vMouseDelta[2] = {x-g_vMousePos[0], y-g_vMousePos[1]};

	switch (g_ControlState)
	{
	case TRANSLATE:  
		if (g_iLeftMouseButton)
		{
			g_vLandTranslate[0] += vMouseDelta[0]*0.01;
			g_vLandTranslate[1] -= vMouseDelta[1]*0.01;
		}
		if (g_iMiddleMouseButton)
		{
			g_vLandTranslate[2] += vMouseDelta[1]*0.01;
		}
		break;
	case ROTATE:
		if (g_iLeftMouseButton)
		{
			g_vLandRotate[0] += vMouseDelta[1];
			g_vLandRotate[1] += vMouseDelta[0];
		}
		if (g_iMiddleMouseButton)
		{
			g_vLandRotate[2] += vMouseDelta[1];
		}
		break;
	case SCALE:
		if (g_iLeftMouseButton)
		{
			g_vLandScale[0] *= 1.0+vMouseDelta[0]*0.01;
			g_vLandScale[1] *= 1.0-vMouseDelta[1]*0.01;
		}
		if (g_iMiddleMouseButton)
		{
			g_vLandScale[2] *= 1.0-vMouseDelta[1]*0.01;
		}
		break;
	}
	g_vMousePos[0] = x;
	g_vMousePos[1] = y;
}
void mouseidle(int x, int y)
{
	g_vMousePos[0] = x;
	g_vMousePos[1] = y;
}
void mousebutton(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		g_iLeftMouseButton = (state==GLUT_DOWN);
		break;
	case GLUT_MIDDLE_BUTTON:
		g_iMiddleMouseButton = (state==GLUT_DOWN);
		break;
	case GLUT_RIGHT_BUTTON:
		g_iRightMouseButton = (state==GLUT_DOWN);
		break;
	}

	switch(glutGetModifiers())
	{
	case GLUT_ACTIVE_CTRL:
		g_ControlState = TRANSLATE;
		break;
	case GLUT_ACTIVE_SHIFT:
		g_ControlState = SCALE;
		break;
	default:
		g_ControlState = ROTATE;
		break;
	}

	g_vMousePos[0] = x;
	g_vMousePos[1] = y;
}
void keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
	
		// Disabled. For test
	case KEY_S:
		g_iTimeStep += 3;
		break;
		// turn off/on lights
	case KEY_L:
		isLightOn = !isLightOn;
		if (isLightOn)
		{
			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);
			glEnable(GL_LIGHT1);
		}
		else
		{
			glDisable(GL_LIGHT0);
			glDisable(GL_LIGHT1);
			glDisable(GL_LIGHTING);
		}
		break;
		// switch between static camera / FPS camera
	case KEY_C:
 		isCameraMovable = !isCameraMovable;
		break;
		// start/stop saving screenshots
	case KEY_A:
		isStartingAnimation = true;
		break;
	default:
		break;
	}
}
// compute the name of next screenshot frame
void getFileName(int id, char filename[])
{
	int foldLen = strlen(folderName);
	strncpy(filename, folderName, foldLen);
	foldLen -= 1;
	for(int i=3; i>0; i--, id/=10)
	{
		char ch = '0'+id%10;
		*(filename+i+foldLen) = ch;
	}
	strncpy(filename+foldLen+3+1, ".jpg", sizeof(".jpg")/sizeof(char));
	*(filename+foldLen+3+1+sizeof(".jpg")/sizeof(char)+1)='\0';
}
// selector for handling save screen shots ( not multiple thread)
void scheduleForScreenShot()
{
	static long int ellipseTime = TPF;
	static long int screenshots = 0;
	static DWORD lastFrameTime = 0;
	DWORD currentTime = GetTickCount();
	//printf("currentTime: %f\n", currentTime * 0.001f);
	ellipseTime -= (currentTime - lastFrameTime);
	lastFrameTime = currentTime;
	//printf("current time: %ld\t ellipseTime: %ld\t \n", currentTime, ellipseTime);
	if(ellipseTime <= 0)
	{
		screenshots++;
		if(screenshots >= MAX_SCREENSHOTS)
		{
			isStartingAnimation = false;
			ellipseTime = TPF;
			screenshots = 0;
			lastFrameTime = 0;
			return;
		}
		char filename[30]={'0'};
		getFileName(screenshots, filename);
		saveScreenshot(filename);
		//printf("Save Screenshot at Time: %f, %d screenshots\n", currentTime * 0.001, screenshots);
		ellipseTime = TPF;
	}
}
// save screen shot to disk file
void saveScreenshot (char *filename)
{
	int i;
	Pic *in = NULL;

	if (filename == NULL)
		return;

	/* Allocate a picture buffer */
	in = pic_alloc(640, 480, 3, NULL);

	printf("File to save to: %s\n", filename);

	for (i=479; i>=0; i--) {
		glReadPixels(0, 479-i, 640, 1, GL_RGB, GL_UNSIGNED_BYTE,
			&in->pix[i*in->nx*in->bpp]);
	}

	if (jpeg_write(filename, in))
		printf("File saved Successfully\n");
	else
		printf("Error in Saving\n");

	pic_free(in);
}

int _tmain(int argc, _TCHAR* argv[])
{
	if (argc<2)
	{  
		printf ("usage: %s <trackfile>\n", argv[0]);
		exit(0);
	}
	loadSplines(argv[1]);
	
	glutInit(&argc,(char**)argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(500, 500);
	glutCreateWindow("Roll Coaster");
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(doIdle);
	glutMotionFunc(mousedrag);
	glutPassiveMotionFunc(mouseidle);
	glutMouseFunc(mousebutton);	
	glutKeyboardFunc(keyboard);
	init();
	glEnable(GL_DEPTH_TEST);
	glutMainLoop();

	return 0;
} 