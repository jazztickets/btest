#include <iostream>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

using namespace std;

static btDefaultCollisionConfiguration *CollisionConfiguration;
static btCollisionDispatcher *Dispatcher;
static btBroadphaseInterface *BroadPhase;
static btSequentialImpulseConstraintSolver *Solver;
static btDiscreteDynamicsWorld *World;


static float TimeStep = 1.0f / 100.0f;
static float TimeStepAccumulator = 0.0f;

int main(int ArgumentCount, char **Arguments) {

	// Set up physics modules
	CollisionConfiguration = new btDefaultCollisionConfiguration();
	//BroadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
	BroadPhase = new btDbvtBroadphase();
	Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	Solver = new btSequentialImpulseConstraintSolver();
	World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);
	World->setGravity(btVector3(0.0f, -9.81f, 0.0f));
	
	World->stepSimulation(TimeStep);
	
	delete World;
	delete Solver;
	delete Dispatcher;
	delete BroadPhase;
	delete CollisionConfiguration;

	return 0;
}
