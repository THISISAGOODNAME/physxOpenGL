/*
=====================================================================

Book				: Learning Physics Modeling with PhysX (ISBN: 978-1-84969-814-6)
Author				: Krishna Kumar
Compiler used		: Visual C++ 2010 Express
PhysX SDK version	: 3.3.0 
Source code name	: ch2_1_HelloPhysx 
Reference Chapter	: Chapter-2: Basic Concepts

Description			: This example demonstrates the initialization, stepping and shutdown of PhysX SDK version 3.3.0
					  It is more like 'Hello World' program for PhysX SDK and contains minimal code. It mainly contains
					  three function which are as follows;

					  void InitPhysX();		//Initialize the PhysX SDK and create two actors. 
					  void StepPhysX();		//Step PhysX simulation 300 times.
					  void ShutdownPhysX(); // Shutdown PhysX SDK

					  ConnectPVD();			//Function for the visualization of PhysX simulation (Optional) 
				  	
					  This example code itself doesn't provide any implementation for rendering the PhysX objects.
					  However you can use PVD software to visualize the simulation. PVD can only be used in 'Debug' mode(configuration).
					  Please refer to last chapter (PhysX Visual Debugger) for more information.

=====================================================================
*/

/*
=====================================================================

Author				: Yanjun Yang
Compiler used		: Apple LLVM version 8.1.0 (clang-802.0.42)clang llvm
PhysX SDK version	: 3.4.0
Reference Chapter	: Chapter-2: Basic Concepts

=====================================================================
*/

#define _DEBUG 1

#include <iostream> 
#include <PxPhysicsAPI.h> //Single header file to include all features of PhysX API


using namespace std;
using namespace physx; 


//--------------Global variables--------------//
static PxPhysics*				gPhysicsSDK = NULL;			//Instance of PhysX SDK
static PxFoundation*			gFoundation = NULL;			//Instance of singleton foundation SDK class
static PxDefaultErrorCallback	gDefaultErrorCallback;		//Instance of default implementation of the error callback
static PxDefaultAllocator		gDefaultAllocatorCallback;	//Instance of default implementation of the allocator interface required by the SDK

PxScene*						gScene = NULL;				//Instance of PhysX Scene				
PxReal							gTimeStep = 1.0f/60.0f;		//Time-step value for PhysX simulation 
PxRigidDynamic					*gBox = NULL;				//Instance of box actor 


//-----------PhysX function prototypes------------//
void InitPhysX();		//Initialize the PhysX SDK and create two actors. 
void StepPhysX();		//Step PhysX simulation 300 times.
void ShutdownPhysX();	//Shutdown PhysX SDK




int main()
{
	
	InitPhysX();  //Initialize PhysX then create scene and actors
	//ConnectPVD(); //Uncomment this function to visualize  the simulation in PVD

	//Simulate PhysX 300 times
	for(int i=0; i<=300; i++) 
	{
		//Step PhysX simulation
		if(gScene) 
			StepPhysX(); 
		 
		//Get current position of actor(box) and print it
		PxVec3 boxPos = gBox->getGlobalPose().p;
		cout<<"Box current Position ("<<boxPos.x <<" "<<boxPos.y <<" "<<boxPos.z<<")\n";
	}

	cout<<"\nSimulation is done, shutting down PhysX!\n";

	//Shut down PhysX
	ShutdownPhysX();

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
	
	
	//Creating PhysX material
	PxMaterial* material = gPhysicsSDK->createMaterial(0.5,0.5,0.5); //Creating a PhysX material

	
	
	//---------Creating actors-----------]
	
	//1-Creating static plane	 
	PxTransform planePos =	PxTransform(PxVec3(0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));	//Position and orientation(transform) for plane actor  
	PxRigidStatic* plane =  gPhysicsSDK->createRigidStatic(planePos);								//Creating rigid static actor	
	plane->createShape(PxPlaneGeometry(), *material);												//Defining geometry for plane actor
	gScene->addActor(*plane);																		//Adding plane actor to PhysX scene


	//2-Creating dynamic cube																		 
	PxTransform		boxPos(PxVec3(0.0f, 10.0f, 0.0f));												//Position and orientation(transform) for box actor 
	PxBoxGeometry	boxGeometry(PxVec3(2,2,2));											//Defining geometry for box actor
					gBox = PxCreateDynamic(*gPhysicsSDK, boxPos, boxGeometry, *material, 1.0f);		//Creating rigid static actor
					gScene->addActor(*gBox);														//Adding box actor to PhysX scene



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

