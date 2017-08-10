
/*
=====================================================================

Book				: Learning Physics Modeling with PhysX (ISBN: 978-1-84969-814-6)
Author				: Krishna Kumar
Compiler used		: Visual C++ 2010 Express
PhysX SDK version	: 3.3.0 
Source code name	: ch7_1_CharController
Reference Chapter	: Chapter-7: Character Controller 

Description			: This example demonstrates how to use Character Controller in PhysX SDK.   
					  	
						
					  Character Movement Controls:-
					  Up Key     : Move char-controller forward
					  Down Key   : Move char-controller backward
					  Left Key   : Move char-controller left
					  Right Key  : Move char-controller right
					   
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
Reference Chapter	: Chapter-7: Character Controller

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


static PxPhysics*				gPhysicsSDK = NULL;			//Instance of PhysX SDK
static PxFoundation*			gFoundation = NULL;			//Instance of singleton foundation SDK class
static PxDefaultErrorCallback	gDefaultErrorCallback;		//Instance of default implementation of the error callback
static PxDefaultAllocator		gDefaultAllocatorCallback;	//Instance of default implementation of the allocator interface required by the SDK
PxScene*						gScene = NULL;				//Instance of PhysX Scene				
PxReal							gTimeStep = 1.0f/60.0f;		//Time-step value for PhysX simulation 

//---for calculating elasped frame time--
int oldTimeSinceStart = 0;
float mAccumulator = 0.0f;

//---Character controller related----
PxControllerManager* gControllerMgr = NULL;			
PxCapsuleController* gCapsuleController = NULL;

PxControllerFilters gCharacterControllerFilters;


float gSpeed 			= 0.5f;			 //Move speed of character controller
float gJumpSpeed 		= 8.0;			 //Jump value of character controller	
float gGravity  		= 10.0;			 //Gravity value of character controller 
PxVec3 gMoveDirection 	= PxVec3(0,0,0); //Move direction of character controller


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
void OnKeyPressed(int,int,int);		//Called when any keyboard button is pressed
void OnKeyReleased(int,int,int);	//Called when any keyboard button is released





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

	glutSpecialFunc(OnKeyPressed);
	glutSpecialUpFunc(OnKeyReleased);

	glutMainLoop();				//Enter the event-processing loop
	atexit(OnShutdown);			//Called on application exit

	return EXIT_SUCCESS;
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

	
	//Creating PhysX material (staticFriction, dynamicFriction, restitution)
	PxMaterial* material = gPhysicsSDK->createMaterial(0.1f,0.1f,0.1f);



	//1)-create static plane	 
	PxTransform planePos =	PxTransform(PxVec3(0.0f, 0, 0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
	PxRigidStatic* plane =  gPhysicsSDK->createRigidStatic(planePos);	

	plane->createShape(PxPlaneGeometry(), *material);
	gScene->addActor(*plane);

	//Creating an inclined flat box for testing the movement of character controller.
	 PxRigidStatic* staticBox  = PxCreateStatic(*gPhysicsSDK, PxTransform(PxVec3(0.0f, 0, 0.0f),PxQuat(PxPi/10, PxVec3(1,0,0))),PxBoxGeometry(15,1,40),*material);
	 gScene->addActor(*staticBox);



	//2)Creating Character controller
	gControllerMgr = PxCreateControllerManager(*gScene);
	
	if(gControllerMgr == NULL)
		cout<<" gControllerMgr failed \n";


	
	//PxBoxControllerDesc playerBoxDesc;

	//playerBoxDesc.position			= PxExtendedVec3(0,10,0);
	//playerBoxDesc.material			= mDynamicMaterial;
	//playerBoxDesc.density 			= 100.0f;
	//playerBoxDesc.halfHeight 			= 5.0f;
	//playerBoxDesc.halfSideExtent		= 1.75f;
	//playerBoxDesc.halfForwardExtent 	= 1.5f;
	//playerBoxDesc.contactOffset 		= 0.05f;
	//playerBoxDesc.slopeLimit 			= 0.2f;
	//playerBoxDesc.stepOffset 			= 0.75f;


	//Defining the propeties of Capsule Controller descriptor class
	PxCapsuleControllerDesc capsuleDesc;
	capsuleDesc.height		= 4; //Height of capsule
	capsuleDesc.radius		= 2; //Radius of casule
	capsuleDesc.position	= PxExtendedVec3(0,20,0); //Initial position of capsule
	capsuleDesc.material	= gPhysicsSDK->createMaterial(0.5f, 0.5f, 0.3f); //Material for capsule shape
	capsuleDesc.density		= 100.0f; //Desity of capsule shape
	capsuleDesc.contactOffset = 0.05f;	
	capsuleDesc.slopeLimit	= 0.2f;
	capsuleDesc.stepOffset	= 0.75f;

	
	if(!capsuleDesc.isValid())
		cout<<"Descriptor is not valid!\n\n";

	gCapsuleController = static_cast<PxCapsuleController*>(gControllerMgr->createController(capsuleDesc));


	if(gCapsuleController == NULL)
		cout<<"gController failed \n";

	//=====================
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

	gMoveDirection *= gSpeed;				  //Speed of character controller
	gMoveDirection.y -= gGravity * deltaTime; //Programmatically applying gravity on character controller

	gCapsuleController->move(gMoveDirection,0.001,deltaTime,gCharacterControllerFilters); //moving the character controller
	
	
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



void OnKeyPressed(int key, int xx, int yy) 
{
	//cout<<"btn state"<<yy <<"\n";

	switch (key) 
	{
		case GLUT_KEY_LEFT : gMoveDirection = PxVec3(-1,0,0); break;
		case GLUT_KEY_RIGHT:gMoveDirection = PxVec3(1,0,0); break;
		case GLUT_KEY_UP   : gMoveDirection = PxVec3(0,0,-1); break;
		case GLUT_KEY_DOWN :gMoveDirection = PxVec3(0,0,1); break;

		case GLUT_KEY_HOME:	gMoveDirection.y = gJumpSpeed;
	}

	glutPostRedisplay();

}

void OnKeyReleased(int key, int x, int y) 
{
	switch (key) 
	{
		//cout<<"btn released"<<"\n";
		case GLUT_KEY_LEFT :
		case GLUT_KEY_RIGHT :
		case GLUT_KEY_UP :
		case GLUT_KEY_DOWN : gMoveDirection = PxVec3(0); break;	
	}
}
