/*
=====================================================================

Book				: Learning Physics Modeling with PhysX (ISBN: 978-1-84969-814-6)
Author				: Krishna Kumar
Compiler used		: Visual C++ 2010 Express
PhysX SDK version	: 3.3.0 
Source code name	: ch3_1_Rigidbody 
Reference Chapter	: Chapter-3: Rigid Body Dynamics

Description			: This example demonstrates how to change the properties of dynamic rigid body in PhysX.
					  A dynamic rigid body can have properties like- mass, density, velocity, acceleration
					  angular-motion and so on. This program also uses 'Freeglut' library which is a cross-platform 
					  windowing toolkit that provides us OpenGL rendering context as well as keyboard and mouse events. 
					  Freeglut can be downloaded from http://freeglut.sourceforge.net	

=====================================================================
*/

/*
=====================================================================

Author				: Yanjun Yang
Compiler used		: Apple LLVM version 8.1.0 (clang-802.0.42)clang llvm
PhysX SDK version	: 3.4.0
Reference Chapter	: Chapter-3: Rigid Body Dynamics

=====================================================================
*/

#define _DEBUG 1

#include <iostream> 
#include <PxPhysicsAPI.h> //Single header file to include all features of PhysX API 
#include <GL/freeglut.h>  //OpenGL window tool kit 
#include "RenderBuffer.h" //Used for rendering PhysX objetcs 



using namespace std;
using namespace physx; 


//--------------Global variables--------------//
static PxPhysics*				gPhysicsSDK = NULL;			//Instance of PhysX SDK
static PxFoundation*			gFoundation = NULL;			//Instance of singleton foundation SDK class
static PxDefaultErrorCallback	gDefaultErrorCallback;		//Instance of default implementation of the error callback
static PxDefaultAllocator		gDefaultAllocatorCallback;	//Instance of default implementation of the allocator interface required by the SDK

PxScene*						gScene = NULL;				//Instance of PhysX Scene				
PxReal							gTimeStep = 1.0f/60.0f;		//Time-step value for PhysX simulation 


PxRigidDynamic*		gBox = NULL;				//Instance of box actor 
PxRigidDynamic*		gConnectedBox = NULL;


float gCamRoateX = 15; 
float gCamRoateY = 0;
float gCamDistance	= -50;

int gWindowWidth  = 800;
int gWindowHeight = 600;

//-----------PhysX function prototypes------------//
void InitPhysX();		//Initialize the PhysX SDK and create actors. 
void StepPhysX();		//Step PhysX simulation
void ShutdownPhysX();	//Shutdown PhysX SDK



//Functions for glut callbacks
void OnRender();			//Display callback for the current glut window
void OnIdle();				//Called whenever the application is idle
void OnReshape(int, int);	//Called whenever the application window is resized
void OnShutdown();			//Called on application exit




int main(int argc, char** argv)
{


	glutInit(&argc, argv);							// Initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);	//Enable double buffering
	glutSetOption(GLUT_MULTISAMPLE, 16);
	glutInitWindowSize(gWindowWidth, gWindowHeight);	//Set window's initial width & height
	
	glutCreateWindow("PhysX and openGL"); // Create a window with the given title

	InitPhysX();

	glutDisplayFunc(OnRender);	//Display callback for the current glut window
	glutIdleFunc(OnIdle);		//Called whenever the application is idle
	glutReshapeFunc(OnReshape); //Called whenever the app window is resized

	//glutMouseFunc(Mouse);		//Called on mouse button event
	//glutMotionFunc(Motion);	//Called on mouse motion event 

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
	PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());	//Descriptor class for scenes 

	sceneDesc.gravity		= PxVec3(0.0f, -9.8f, 0.0f);		//Setting gravity
	sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(1);	//Creates default CPU dispatcher for the scene
	sceneDesc.filterShader  = PxDefaultSimulationFilterShader;	//Creates default collision filter shader for the scene
	
	gScene = gPhysicsSDK->createScene(sceneDesc);				//Creates a scene 

	//This will enable basic visualization of PhysX objects like- actors collision shapes and it's axis. 
	//The function PxScene::getRenderBuffer() is used to render any active visualization for scene.
	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE,				1.0);	//Global visualization scale which gets multiplied with the individual scales
	gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES,	1.0f);	//Enable visualization of actor's shape
	gScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES,		1.0f);	//Enable visualization of actor's axis
	
	gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS,		1.0f);
	gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES,1.0f);
	

	//Creating PhysX material (staticFriction, dynamicFriction, restitution)
	PxMaterial* material = gPhysicsSDK->createMaterial(0.5f,0.5f,0.5f);

	
	
	//---------Creating actors-----------]
	
	//1-Creating static plane	 
	PxTransform planePos =	PxTransform(PxVec3(0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));	//Position and orientation(transform) for plane actor  
	PxRigidStatic* plane =  gPhysicsSDK->createRigidStatic(planePos);								//Creating rigid static actor	
	plane->createShape(PxPlaneGeometry(), *material);												//Defining geometry for plane actor
	gScene->addActor(*plane);																		//Adding plane actor to PhysX scene


	
	//2-Creating dynamic cube
	//We will apply a force on this created actor in 'OnRender()' function
	{
		PxMaterial* mat = gPhysicsSDK->createMaterial(0.2f,0.2f,0.2f);
		PxTransform		boxPos(PxVec3(0.0f, 10.0f, 30.0f));												
		PxBoxGeometry	boxGeometry(PxVec3(1.5f,1.5f,1.5f));											
						gBox = PxCreateDynamic(*gPhysicsSDK, boxPos, boxGeometry, *mat, 1.0f);		
						gScene->addActor(*gBox);														
	
	}

	//3-Creating dynamic cube 
	//We will apply initial linear velocity on the actor that will push actor upward for a while
	{																					 
		PxTransform		boxPos(PxVec3(5.0f, 0.1f, 0.0f));											
		PxBoxGeometry	boxGeometry(PxVec3(1.5f,1.5f,1.5f));										
		PxRigidDynamic* box2 = PxCreateDynamic(*gPhysicsSDK, boxPos, boxGeometry, *material, 1.0f);		
						
						box2->setMass(1);							//Setting mass of the actor
						box2->setLinearVelocity(PxVec3(0,25,0));	//Setting initial linear velocity on the actor (This will push actor upward for a while)
						gScene->addActor(*box2);
	}

	//4-Creating a box actor attached with a static actor that is free to rotate around y axis.
	//The box actor will keep rotating around y axis because we are applying angular velocity on it in 'OnRender()' function 
	{
		PxVec3 pos = PxVec3(10,15,25); 
		PxVec3 offset = PxVec3(0,1.5,0);

		PxRigidActor* staticActor = PxCreateStatic(*gPhysicsSDK, PxTransform(pos), PxSphereGeometry(0.5f), *material);
		 gConnectedBox = PxCreateDynamic(*gPhysicsSDK, PxTransform(PxVec3(0),PxQuat(PxHalfPi,PxVec3(0,0,1))), PxBoxGeometry(0.5,0.5,4), *material, 1.0f);
	
		PxD6Joint* d6Joint = PxD6JointCreate(*gPhysicsSDK, staticActor, PxTransform(-offset), gConnectedBox, PxTransform(offset));
				   d6Joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
				   d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);

		gScene->addActor(*staticActor);
		gScene->addActor(*gConnectedBox);

	}

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
	//Update PhysX	
	if(gScene) 
		StepPhysX(); 

	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	
	glTranslatef(0,0,gCamDistance);
	glRotatef(gCamRoateX,1,0,0);
	glRotatef(gCamRoateY,0,1,0);

	RenderData(gScene->getRenderBuffer());

	gConnectedBox->setAngularVelocity(PxVec3(0,1,0)); //Applying angular velocity to the actor
	gBox->addForce(PxVec3(0,0,-180));				  //Applying force to the actor		

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



