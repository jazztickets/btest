#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btInternalEdgeUtility.h>
#include <BulletCollision/CollisionShapes/btTriangleMeshShape.h>
#include <cstdio>

// culprit: Solver->m_tmpSolverBodyPool.m_data[1].m_deltaLinearVelocity.m_floats[0]
// set at btSequentialImpulseConstraintSolver.cpp:1374

// similar issue: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=8113&hilit=internal

const bool USE_INTERNAL_EDGE_UTILITY = true;
const bool USE_STATIC_BOX = false;
const btScalar TIMESTEP = 1.0 / 100.0;

static btDefaultCollisionConfiguration *CollisionConfiguration;
static btCollisionDispatcher *Dispatcher;
static btBroadphaseInterface *BroadPhase;
static btSequentialImpulseConstraintSolver *Solver;
static btDiscreteDynamicsWorld *World;

static btRigidBody *SphereBody = NULL;
static btRigidBody *BoxBody = NULL;
static btRigidBody *MeshBody = NULL;
static btTriangleIndexVertexArray *TriangleIndexVertexArray = NULL;
static btTriangleInfoMap *TriangleInfoMap = NULL;

// Used for conditional breakpoints
static bool HitOnce = false;

static btScalar Vertices[] = {
	2.000000, 0.000000, 2.000000,
	-2.000000, 0.000000, 2.000000,
	2.000000, 0.000000, -2.000000,
	-2.000000, 0.000000, -2.000000,
};

static int Indices[] = {
	1, 0, 2,
	3, 1, 2,
};

static bool CustomMaterialCallback(btManifoldPoint &ManifoldPoint, const btCollisionObjectWrapper *Object0, int PartID0, int Index0, const btCollisionObjectWrapper *Object1, int PartID1, int Index1) {

	if(Object1->getCollisionShape()->getShapeType() != TRIANGLE_SHAPE_PROXYTYPE)
		return false;
		
	btScalar Before = ManifoldPoint.m_normalWorldOnB.getY();
	if(USE_INTERNAL_EDGE_UTILITY) {
		HitOnce = true;
		btAdjustInternalEdgeContacts(ManifoldPoint, Object1, Object0, PartID1, Index1);
	}
	btScalar After = ManifoldPoint.m_normalWorldOnB.getY();
	if(Before != After) printf("contact callback: before y=%f after y=%f\n", Before, After);

	return false;
}

int main(int ArgumentCount, char **Arguments) {

	// Set up physics modules
	CollisionConfiguration = new btDefaultCollisionConfiguration();
	//BroadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
	BroadPhase = new btDbvtBroadphase();
	Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	Solver = new btSequentialImpulseConstraintSolver();
	World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);
	World->setGravity(btVector3(0.0, -9.81, 0.0));
	btContactSolverInfo &SolverInfo = World->getSolverInfo();
	//SolverInfo.m_splitImpulseTurnErp = 0.0f;
	//SolverInfo.m_splitImpulse = 1;
	//SolverInfo.m_splitImpulsePenetrationThreshold = -0.02f;
	
	gContactAddedCallback = CustomMaterialCallback;
	
	{
		btScalar Mass = 1.0;
		btCollisionShape *Shape = new btSphereShape(0.5);
		btVector3 LocalInertia(0.0, 0.0, 0.0);
		Shape->calculateLocalInertia(Mass, LocalInertia);
		
		SphereBody = new btRigidBody(Mass, NULL, Shape, LocalInertia);
		SphereBody->setFriction(1.0);
		SphereBody->setDamping(0.1, 0.3);
		SphereBody->setActivationState(DISABLE_DEACTIVATION);
		SphereBody->translate(btVector3(0.71, 2.5, -0.8));
		
		World->addRigidBody(SphereBody);
	}
	
	// Set to 1 to test static box shape
	if(USE_STATIC_BOX) {
		
		btScalar Mass = 0.0;
		btCollisionShape *Shape = new btBoxShape(btVector3(1, 1, 1));
		btVector3 LocalInertia(0.0, 0.0, 0.0);
		Shape->calculateLocalInertia(Mass, LocalInertia);
		
		BoxBody = new btRigidBody(Mass, NULL, Shape, LocalInertia);
		BoxBody->setFriction(1.0);
		BoxBody->setDamping(0.1, 0.3);
		BoxBody->setActivationState(DISABLE_DEACTIVATION);
		BoxBody->translate(btVector3(0, -2, 0));
		
		World->addRigidBody(BoxBody);
	}
	else {
		btScalar Mass = 0.0;
		int VertexCount = sizeof(Vertices) / sizeof(btScalar);
		int FaceCount = sizeof(Indices) / sizeof(int) / 3;
		TriangleIndexVertexArray = new btTriangleIndexVertexArray(FaceCount, Indices, 3 * sizeof(int), VertexCount, Vertices, 3 * sizeof(btScalar));

		btBvhTriangleMeshShape *Shape = new btBvhTriangleMeshShape(TriangleIndexVertexArray, true);
		btTriangleInfoMap *TriangleInfoMap = new btTriangleInfoMap();
		btGenerateInternalEdgeInfo(Shape, TriangleInfoMap);
	
		MeshBody = new btRigidBody(Mass, NULL, Shape);
		MeshBody->setFriction(1.0);
		MeshBody->setDamping(0.1, 0.3);
		MeshBody->setCollisionFlags(MeshBody->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

		// Add body
		World->addRigidBody(MeshBody);
	}
	
	for(int i = 0; i < 100; i++) {
		World->stepSimulation(TIMESTEP, 0, 0);
	
		const btTransform &Transform = SphereBody->getCenterOfMassTransform();
		const btVector3 &Position = Transform.getOrigin();
		printf("%f %f %f\n", Position.getX(), Position.getY(), Position.getZ());
	}
	
	World->removeRigidBody(SphereBody);
	if(MeshBody)
		World->removeRigidBody(MeshBody);
	if(BoxBody)
		World->removeRigidBody(BoxBody);
		
	delete SphereBody->getCollisionShape();	
	delete SphereBody;

	if(MeshBody) {
		delete ((btBvhTriangleMeshShape *)MeshBody->getCollisionShape())->getTriangleInfoMap();	
		delete ((btBvhTriangleMeshShape *)MeshBody->getCollisionShape())->getMeshInterface();	
		delete MeshBody->getCollisionShape();	
		delete MeshBody;
	}
	
	if(BoxBody) {
		delete BoxBody->getCollisionShape();	
		delete BoxBody;
	}
	
	delete World;
	delete Solver;
	delete Dispatcher;
	delete BroadPhase;
	delete CollisionConfiguration;

	return 0;
}
