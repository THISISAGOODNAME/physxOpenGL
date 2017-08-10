
/*
=====================================================================

Book				: Learning Physics Modeling with PhysX (ISBN: 978-1-84969-814-6)
Author				: Krishna Kumar
Compiler used		: Visual C++ 2010 Express
PhysX SDK version	: 3.3.0 
Source code name	: ch9_1_Cloth
Reference Chapter	: Chapter-9: Cloth 

Description			: This example demonstrates how to create cloth in PhysX SDK. Cloths are used for
 					  simulating cloth behavior of a game character or for simulating realistic behavior
					  of deformable objects like- flags and curtains in a game environment. 		     	     
					  	
					  	
					  Navigation Controls:-
					  Mouse Left-Button Drag  : Scene orbital navigation
					  Mouse Right-Button Drag : Scene zoom-in/zoom-out
				
=====================================================================
*/

/*
=====================================================================

Author				: Yanjun Yang
Compiler used		: Apple LLVM version 8.1.0 (clang-802.0.42)clang llvm
PhysX SDK version	: 3.4.0
Reference Chapter	: Chapter-9: Cloth

=====================================================================
*/

#define _DEBUG 1

#include <iostream> 
#include <PxPhysicsAPI.h> //Single header file to include all features of PhysX API 
#include <GL/freeglut.h>  //OpenGL window tool kit 
#include "RenderBuffer.h"	  //Used for rendering PhysX objetcs 



using namespace std;
using namespace physx; 


//========== Global variables ============//

int gWindowWidth  = 800; //Screen width
int gWindowHeight = 600; //Screen height


//---Scene navigation----
int gOldMouseX = 0;
int gOldMouseY = 0;

bool isMouseLeftBtnDown  = false;
bool isMouseRightBtnDown = false;

float gCamRoateX	= 15; 
float gCamRoateY	= 0;
float gCamDistance	= -50;
//------------------------

int oldTimeSinceStart = 0;
float mAccumulator = 0.0f;


static PxPhysics*				gPhysicsSDK = NULL;			//Instance of PhysX SDK
static PxFoundation*			gFoundation = NULL;			//Instance of singleton foundation SDK class
static PxDefaultErrorCallback	gDefaultErrorCallback;		//Instance of default implementation of the error callback
static PxDefaultAllocator		gDefaultAllocatorCallback;	//Instance of default implementation of the allocator interface required by the SDK
PxScene*						gScene = NULL;				//Instance of PhysX Scene				
PxReal							gTimeStep = 1.0f/60;		//Time-step value for PhysX simulation 

PxCloth* gCloth = NULL;										//Intance of cloth


//========== PhysX function prototypes ===========//

void InitPhysX();		//Initialize the PhysX SDK and create actors. 
void StepPhysX();		//Step PhysX simulation
void ShutdownPhysX();	//Shutdown PhysX SDK



//Functions for glut callbacks
void OnRender();					//Display callback for the current glut window
void OnIdle();						//Called whenever the application is idle
void OnReshape(int, int);			//Called whenever the application window is resized
void OnShutdown();					//Called on application exit
void OnMouseMotion(int,int);		//Called when the mouse is moving
void OnMousePress(int,int,int,int); //Called when any mouse button is pressed




int main(int argc, char** argv)
{

    glutInit(&argc, argv);								//Initialize GLUT
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);		//Enable double buffering
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);		//Enable double buffering
    glutSetOption(GLUT_MULTISAMPLE, 16);
    glutInitWindowSize(gWindowWidth, gWindowHeight);	//Set window's initial width & height

    glutCreateWindow("PhysX and openGL"); // Create a window with the given title

    InitPhysX();

    glutDisplayFunc(OnRender);	//Display callback for the current glut window
    glutIdleFunc(OnIdle);		//Called whenever the application is idle
    glutReshapeFunc(OnReshape); //Called whenever the app window is resized

    glutMouseFunc(OnMousePress);	//Called on mouse button event
    glutMotionFunc(OnMouseMotion);	//Called on mouse motion event

    glutMainLoop();				//Enter the event-processing loop
    atexit(OnShutdown);			//Called on application exit

    return EXIT_SUCCESS;
}


void CreateCloth()
{
	
	PxTransform gPose = PxTransform(PxVec3(0,1,0));
	
	// create regular mesh
	PxU32 resolution = 20;
	PxU32 numParticles = resolution*resolution;
	PxU32 numTriangles = 2*(resolution-1)*(resolution-1);

	// create cloth particles
	PxClothParticle* particles = new PxClothParticle[numParticles];
	PxVec3 center(0.5f, 0.3f, 0.0f);
	PxVec3 delta = 1.0f/(resolution-1) * PxVec3(15.0f, 15.0f, 15.0f);
	PxClothParticle* pIt = particles;
	for(PxU32 i=0; i<resolution; ++i)
	{
		for(PxU32 j=0; j<resolution; ++j, ++pIt)
		{
			pIt->invWeight = j+1<resolution ? 1.0f : 0.0f;
			pIt->pos = delta.multiply(PxVec3(PxReal(i), 
				PxReal(j), -PxReal(j))) - center;
		}
	}

	// create triangles
	PxU32* triangles = new PxU32[3*numTriangles];
	PxU32* iIt = triangles;
	for(PxU32 i=0; i<resolution-1; ++i)
	{
		for(PxU32 j=0; j<resolution-1; ++j)
		{
			PxU32 odd = j&1u, even = 1-odd;
			*iIt++ = i*resolution + (j+odd);
			*iIt++ = (i+odd)*resolution + (j+1);
			*iIt++ = (i+1)*resolution + (j+even);
			*iIt++ = (i+1)*resolution + (j+even);
			*iIt++ = (i+even)*resolution + j;
			*iIt++ = i*resolution + (j+odd);
		}
	}

	// create fabric from mesh
	PxClothMeshDesc meshDesc;
	meshDesc.points.count = numParticles;
	meshDesc.points.stride = sizeof(PxClothParticle);
	meshDesc.points.data = particles;

	meshDesc.invMasses.count = numParticles;
	meshDesc.invMasses.stride = sizeof(PxClothParticle);
	meshDesc.invMasses.data = &particles->invWeight;
	
	meshDesc.triangles.count = numTriangles;
	meshDesc.triangles.stride = 3*sizeof(PxU32);
	meshDesc.triangles.data = triangles;


	// cook fabric
	PxClothFabric* fabric = PxClothFabricCreate(*gPhysicsSDK, meshDesc, PxVec3(0, 1, 0));

	delete[] triangles;

	// create cloth
	gCloth = gPhysicsSDK->createCloth(gPose, *fabric, particles, PxClothFlags(0));

	fabric->release();
	delete[] particles;

	// 240 iterations per/second (4 per-60hz frame)
	gCloth->setSolverFrequency(240.0f);

	gScene->addActor(*gCloth);
		

}



void InitPhysX() 
{
	//Creating foundation for PhysX
	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
	
	//Creating instance of PhysX SDK
	gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale() );

	if(gPhysicsSDK == NULL) 
	{
		cerr<<"Error creating PhysX3 device, Exiting..."<<endl;
		exit(1);
	}



	//Creating scene
	PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());		//Descriptor class for scenes 

	sceneDesc.gravity		= PxVec3(0.0f, -9.8f, 0.0f);			//Setting gravity
	sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(1);		//Creating default CPU dispatcher for the scene
	sceneDesc.filterShader  = PxDefaultSimulationFilterShader;		//Creating default collision filter shader for the scene
	
	gScene = gPhysicsSDK->createScene(sceneDesc);					//Creating a scene 

	
	
	//This will enable basic visualization of PhysX objects like- actors collision shapes and their axes. 
	//The function PxScene::getRenderBuffer() is used to render any active visualization for scene.
	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE,				1.0);	//Global visualization scale which gets multiplied with the individual scales
	gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES,	1.0f);	//Enable visualization of actor's shape
	gScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES,		1.0f);	//Enable visualization of actor's axis

	gScene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_HORIZONTAL,		1.0f);
	gScene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_VERTICAL,1.0f);
	gScene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_BENDING,1.0f);
	
	
	//Creating PhysX material (staticFriction, dynamicFriction, restitution)
	PxMaterial* material = gPhysicsSDK->createMaterial(0.5f,0.5f,0.5f);


	//1)-create static plane	 
	PxTransform planePos =	PxTransform(PxVec3(0, 0, 0),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
	PxRigidStatic* plane =  gPhysicsSDK->createRigidStatic(planePos);	
	
	plane->createShape(PxPlaneGeometry(), *material);
	gScene->addActor(*plane);

	CreateCloth();

	gCloth->addCollisionPlane(PxClothCollisionPlane(PxVec3(0, 1, 0), 0.0f));
	gCloth->addCollisionConvex(1 << 0); // Convex references the first plane

}


void StepPhysX()					//Stepping PhysX
{ 
	
	gScene->simulate(gTimeStep);	//Advances the simulation by 'gTimeStep' time
	gScene->fetchResults(true);		//Block until the simulation run is completed
} 


void ShutdownPhysX()				//Shutdown PhysX
{
	gPhysicsSDK->release();			//Removes any actors,  particle systems, and constraint shaders from this scene
	gFoundation->release();			//Destroys the instance of foundation SDK
}




void OnRender() 
{
	 int timeSinceStart = glutGet(GLUT_ELAPSED_TIME); 
     float deltaTime = (timeSinceStart - oldTimeSinceStart)/1000.0f;
     oldTimeSinceStart = timeSinceStart;
	
	 mAccumulator  += deltaTime;


	while(mAccumulator > gTimeStep) 
	{
		mAccumulator -= gTimeStep;
		StepPhysX(); 
	}
	
	//Update PhysX	
	if(gScene) 
		StepPhysX(); 

	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	
	glTranslatef(0,0,gCamDistance);
	glRotatef(gCamRoateX,1,0,0);
	glRotatef(gCamRoateY,0,1,0);
	

	RenderData(gScene->getRenderBuffer());
	
	glutSwapBuffers();
}


void OnReshape(int w, int h) 
{
	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)w / (GLfloat)h, 0.1f, 100000.0f);
	glMatrixMode(GL_MODELVIEW);
}

void OnIdle() 
{
	glutPostRedisplay();
}

void OnShutdown() 
{
	ShutdownPhysX();
}



void OnMouseMotion(int curMouseX, int curMouseY) 
{
	if(isMouseLeftBtnDown)
	{
		 gCamRoateY += (curMouseX - gOldMouseX)/5.0f;
		 gCamRoateX += (curMouseY - gOldMouseY)/5.0f;
	}

	if(isMouseRightBtnDown)
	{
		gCamDistance -= (curMouseY - gOldMouseY)/5.0f;
	}

	gOldMouseX = curMouseX;
	gOldMouseY = curMouseY;

}


void OnMousePress(int mouseBtn, int mouseBtnState, int curMouseX, int curMouseY)
{
	if (mouseBtnState == GLUT_DOWN) 
	{
		if(mouseBtn== GLUT_LEFT_BUTTON) 
			isMouseLeftBtnDown = true;
		
		else if(mouseBtn == GLUT_RIGHT_BUTTON)
			isMouseRightBtnDown = true;
		
		gOldMouseX = curMouseX;
		gOldMouseY = curMouseY;

	}

	if (mouseBtnState == GLUT_UP ) 
	{
	  	isMouseLeftBtnDown = false;
		isMouseRightBtnDown = false;
	}

	//cout<<mouseBtn<<" "<<mouseBtnState<<" "<<x<<"|"<<y<<"\n";
}
