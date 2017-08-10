/*
=====================================================================

Book				: Learning Physics Modeling with PhysX (ISBN: 978-1-84969-814-6)
Author				: Krishna Kumar
Compiler used		: Visual C++ 2010 Express
PhysX SDK version	: 3.3.0 
Source code name	: ch6_1_SceneQueries
Reference Chapter	: Chapter-6: Scene Queries

Description			: This example demonstrates how to perform scene queries like-Raycast query in PhysX SDK. 
						
					  	
	
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
Reference Chapter	: Chapter-6: Scene Queries

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
PxReal							gTimeStep = 1.0f/60.0f;		//Time-step value for PhysX simulation 

PxRigidDynamic* gBox = NULL;
PxRigidDynamic* gSphere = NULL;

PxReal rotateFactor = 0;
PxRaycastBuffer gRaycastBuffer; //Buffer to store raycast hit information

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


//void RaycastAny()
//{
//	PxVec3 origin = PxVec3(0,3,0);   //[in] Ray origin
//	PxVec3 unitDir = PxVec3(0,1,0);  //[in] Normalized ray direction
//	PxReal maxDistance = 1000;    	 //[in] Raycast max distance
//	PxRaycastBuffer hit;             //[out] Raycast results
//
//	PxQueryFilterData filterData; 
//	filterData.flags |= PxQueryFlag::eANY_HIT;
//
//	bool status = gScene->raycast(origin, unitDir, maxDistance, hit, PxHitFlags(PxHitFlag::eDEFAULT) ,filterData);
//	cout<<"Result "<<status <<"\n";
//
//}







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

	
	
	//---------Creating actors-----------]
	
	//1-Creating static plane that will act as ground	 
	PxTransform planePos =	PxTransform(PxVec3(0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));	//Position and orientation(transform) for plane actor  
	PxRigidStatic* plane =  gPhysicsSDK->createRigidStatic(planePos);								//Creating rigid static actor	
							plane->createShape(PxPlaneGeometry(), *material);						//Defining geometry for plane actor
							gScene->addActor(*plane);												//Adding plane actor to PhysX scene

	
	//Creating a kinematic box actor that will keep rotating around z axis.
	gBox = PxCreateDynamic(*gPhysicsSDK, PxTransform(PxVec3(0,8,0)), PxBoxGeometry(2,5,2), *material,1);
//	gBox->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
	gScene->addActor(*gBox);

	//Creating a kinematic sphere actor to represent raycast hit position.
	PxShape* sphereShape = gPhysicsSDK->createShape(PxSphereGeometry(1), *material);
	sphereShape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false); //Don't consider this shape for hit queries

	gSphere = PxCreateDynamic(*gPhysicsSDK, PxTransform(PxVec3(0)),*sphereShape,1);
//	gSphere->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
	
	gScene->addActor(*gSphere);


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
	//Calculating 'deltaTime' for each frame
	int timeSinceStart = glutGet(GLUT_ELAPSED_TIME); 
    float deltaTime = (timeSinceStart - oldTimeSinceStart)/1000.0f;
    oldTimeSinceStart = timeSinceStart;
	
	mAccumulator  += deltaTime;


	while(mAccumulator > gTimeStep) //Simulate at not more than 'gTimeStep' time-interval 
	{
		mAccumulator -= gTimeStep;
		StepPhysX();
	}

	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	
	glTranslatef(0,0,gCamDistance);
	glRotatef(gCamRoateX,1,0,0);
	glRotatef(gCamRoateY,0,1,0);
	
	RenderData(gScene->getRenderBuffer());
	

	gBox->setGlobalPose(PxTransform(0,5,0,PxQuat(rotateFactor,PxVec3(0,0,1)))); //Rotate box around z axis
	rotateFactor+= deltaTime*1.2f;
	
	
	//-----Cast a ray in downward direction from (0,15,0) position------//

	PxVec3 origin = PxVec3(0,15,0);  //[in] Ray origin
	PxVec3 unitDir = PxVec3(0,-1,0); //[in] Normalized ray direction
	PxReal maxDistance = 1000.0f;    //[in] Raycast max distance
	PxRaycastBuffer gRaycastBuffer;  //[out] Raycast results

	bool status = gScene->raycast(origin, unitDir, maxDistance, gRaycastBuffer); //cast a ray

	if(status)
	{	//On raycast hit, position the sphere actor at current hit position  
		PxVec3 hitPos = gRaycastBuffer.getAnyHit(0).position;
		gSphere->setGlobalPose(PxTransform(hitPos.x,hitPos.y,hitPos.z));
	}

		
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
