#include <iostream>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

using namespace std;

static btDefaultCollisionConfiguration *CollisionConfiguration;
static btCollisionDispatcher *Dispatcher;
static btBroadphaseInterface *BroadPhase;
static btSequentialImpulseConstraintSolver *Solver;
static btDiscreteDynamicsWorld *World;

static btRigidBody *SphereBody = NULL;
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
	
	{
		float Mass = 1.0f;
		btCollisionShape *Shape = new btSphereShape(0.5f);
		btVector3 LocalInertia(0.0f, 0.0f, 0.0f);
		Shape->calculateLocalInertia(Mass, LocalInertia);
		
		SphereBody = new btRigidBody(Mass, NULL, Shape, LocalInertia);
		SphereBody->setFriction(1.0f);
		SphereBody->setDamping(0.1f, 0.3f);
		SphereBody->setSleepingThresholds(0.2f, 0.2f);
		SphereBody->setActivationState(DISABLE_DEACTIVATION);

		// Add body
		World->addRigidBody(SphereBody);
	}
	
	for(int i = 0; i < 100; i++) {
		World->stepSimulation(TimeStep);
	
		const btTransform &Transform = SphereBody->getCenterOfMassTransform();
		const btVector3 &Position = Transform.getOrigin();
		printf("%f %f %f\n", Position.getX(), Position.getY(), Position.getZ());
	}
	
	delete SphereBody->getCollisionShape();	
	delete SphereBody;
	
	delete World;
	delete Solver;
	delete Dispatcher;
	delete BroadPhase;
	delete CollisionConfiguration;

	return 0;
}
