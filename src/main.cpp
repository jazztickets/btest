#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btInternalEdgeUtility.h>
#include <BulletCollision/CollisionShapes/btTriangleMeshShape.h>
#include <cstdio>

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

static btScalar TimeStep = 1.0f / 100.0f;

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
	if(1) {
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
	World->setGravity(btVector3(0.0f, -9.81f, 0.0f));
	
	gContactAddedCallback = CustomMaterialCallback;
	
	{
		btScalar Mass = 1.0f;
		btCollisionShape *Shape = new btSphereShape(0.5f);
		btVector3 LocalInertia(0.0f, 0.0f, 0.0f);
		Shape->calculateLocalInertia(Mass, LocalInertia);
		
		SphereBody = new btRigidBody(Mass, NULL, Shape, LocalInertia);
		SphereBody->setFriction(1.0f);
		SphereBody->setDamping(0.1f, 0.3f);
		SphereBody->setActivationState(DISABLE_DEACTIVATION);
		SphereBody->translate(btVector3(0.71f, 2.5, -0.8f));
		
		World->addRigidBody(SphereBody);
	}
	
	// Set to 1 to test static box shape
	if(0) {
		
		btScalar Mass = 0.0f;
		btCollisionShape *Shape = new btBoxShape(btVector3(1, 1, 1));
		btVector3 LocalInertia(0.0f, 0.0f, 0.0f);
		Shape->calculateLocalInertia(Mass, LocalInertia);
		
		BoxBody = new btRigidBody(Mass, NULL, Shape, LocalInertia);
		BoxBody->setFriction(1.0f);
		BoxBody->setDamping(0.1f, 0.3f);
		BoxBody->setActivationState(DISABLE_DEACTIVATION);
		BoxBody->translate(btVector3(0, -2, 0));
		
		World->addRigidBody(BoxBody);
	}
	else {
		btScalar Mass = 0.0f;
		int VertexCount = sizeof(Vertices) / sizeof(btScalar);
		int FaceCount = sizeof(Indices) / sizeof(int) / 3;
		TriangleIndexVertexArray = new btTriangleIndexVertexArray(FaceCount, Indices, 3 * sizeof(int), VertexCount, Vertices, 3 * sizeof(btScalar));

		btBvhTriangleMeshShape *Shape = new btBvhTriangleMeshShape(TriangleIndexVertexArray, true);
		btTriangleInfoMap *TriangleInfoMap = new btTriangleInfoMap();
		btGenerateInternalEdgeInfo(Shape, TriangleInfoMap);
	
		btVector3 LocalInertia(0.0f, 0.0f, 0.0f);
		Shape->calculateLocalInertia(Mass, LocalInertia);
		
		MeshBody = new btRigidBody(Mass, NULL, Shape, LocalInertia);
		MeshBody->setFriction(1.0f);
		MeshBody->setDamping(0.1f, 0.3f);
		MeshBody->setCollisionFlags(MeshBody->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

		// Add body
		World->addRigidBody(MeshBody);
	}
	
	for(int i = 0; i < 100; i++) {
		World->stepSimulation(TimeStep);
	
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
