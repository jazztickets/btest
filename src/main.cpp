#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btInternalEdgeUtility.h>
#include <BulletCollision/CollisionShapes/btTriangleMeshShape.h>
#include <ode/ode.h>
#include <iostream>
#include <iomanip>
#include <limits>

// similar issue: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=8113&hilit=internal

const double TIMESTEP = 1.0 / 100.0;
const bool BULLET_USE_INTERNAL_EDGE_UTILITY = true;
const bool BULLET_USE_STATIC_BOX = false;
const bool BULLET_TEST_DETERMINISM = 1;

static dWorldID World;
static dJointGroupID ContactGroup;

static void ODECallback(void *Data, dGeomID Geometry1, dGeomID Geometry2) {
	dBodyID Body1 = dGeomGetBody(Geometry1);
	dBodyID Body2 = dGeomGetBody(Geometry2);

	const int MAX_CONTACTS = 32;
	dContact Contacts[MAX_CONTACTS];
	int Count = dCollide(Geometry1, Geometry2, MAX_CONTACTS, &Contacts[0].geom, sizeof(dContact));
	for(int i = 0; i < Count; i++) {
		Contacts[i].surface.mode = dContactApprox1;
		Contacts[i].surface.mu = 1;

		dJointID Joint = dJointCreateContact(World, ContactGroup, &Contacts[i]);
		dJointAttach(Joint, Body1, Body2);
	}
}

static void TestODE() {
	dInitODE();

	// Create world
	World = dWorldCreate();
	dSpaceID Space = dHashSpaceCreate(0);
	dWorldSetGravity(World, 0, -9.81, 0);
	dWorldSetCFM(World, 0);
	dWorldSetERP(World, 0.0);
	dCreatePlane(Space, 0, 1, 0, 0);
	ContactGroup = dJointGroupCreate(0);

	// Create objects
	dBodyID SphereBody;
	{
		SphereBody = dBodyCreate(World);
		dGeomID Geometry = dCreateSphere(Space, 0.5);
		dMass Mass;

		// Set mass
		dMassSetSphereTotal(&Mass, 1.0, 0.5);
		dBodySetMass(SphereBody, &Mass);
		dGeomSetBody(Geometry, SphereBody);

		// Set initial position
		dBodySetPosition(SphereBody, 0.0, 2.5, 0);
	}

	dBodyID BoxBody;
	{
		BoxBody = dBodyCreate(World);
		dGeomID Geometry = dCreateBox(Space, 1, 1, 1);
		dMass Mass;

		// Set mass
		dMassSetBoxTotal(&Mass, 1.0, 1, 1, 1);
		dBodySetMass(BoxBody, &Mass);
		dGeomSetBody(Geometry, BoxBody);

		// Set initial position
		dBodySetPosition(BoxBody, 0.9, 0.5, 0);
	}

	// Run simulation
	double Timer = 0;
	int Steps = 1 / TIMESTEP;
	for(int i = 0; i < Steps; i++) {
		Timer += TIMESTEP;
		dSpaceCollide(Space, 0, &ODECallback);
		dWorldQuickStep(World, TIMESTEP);
		dJointGroupEmpty(ContactGroup);

		const dReal *Position = dBodyGetPosition(SphereBody);
		//const dReal *Position = dBodyGetPosition(BoxBody);
		std::cout << std::setprecision(16) << "t=" << Timer << " " << Position[0] << " " << Position[1] << " " << Position[2] << std::endl;
	}

	// Clean up
	dJointGroupDestroy(ContactGroup);
	dSpaceDestroy(Space);
	dWorldDestroy(World);
	dCloseODE();
}

static bool CustomMaterialCallback(btManifoldPoint &ManifoldPoint, const btCollisionObjectWrapper *Object0, int PartID0, int Index0, const btCollisionObjectWrapper *Object1, int PartID1, int Index1) {

	if(Object1->getCollisionShape()->getShapeType() != TRIANGLE_SHAPE_PROXYTYPE)
		return false;

	btScalar Before = ManifoldPoint.m_normalWorldOnB.getY();
	if(BULLET_USE_INTERNAL_EDGE_UTILITY) {
		btAdjustInternalEdgeContacts(ManifoldPoint, Object1, Object0, PartID1, Index1);
	}

	btScalar After = ManifoldPoint.m_normalWorldOnB.getY();
	if(Before != After)
		std::cout << "contact callback: before y=" << Before << " after y=" << After << std::endl;

	return false;
}

static void TestBullet() {
	btDefaultCollisionConfiguration *CollisionConfiguration;
	btCollisionDispatcher *Dispatcher;
	btBroadphaseInterface *BroadPhase;
	btSequentialImpulseConstraintSolver *Solver;
	btDiscreteDynamicsWorld *World;

	btRigidBody *SphereBody = NULL;
	btRigidBody *BoxBody = NULL;
	btRigidBody *MeshBody = NULL;
	btTriangleIndexVertexArray *TriangleIndexVertexArray = NULL;
	btTriangleInfoMap *TriangleInfoMap = NULL;

	static btScalar Vertices[] = {
		200.000000, 0.000000, 200.000000,
		-200.000000, 0.000000, 200.000000,
		200.000000, 0.000000, -200.000000,
		-200.000000, 0.000000, -200.000000,
	};

	static int Indices[] = {
		1, 0, 2,
		3, 1, 2,
	};

	// Set up physics modules
	CollisionConfiguration = new btDefaultCollisionConfiguration();
	//BroadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
	BroadPhase = new btDbvtBroadphase();
	Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	Solver = new btSequentialImpulseConstraintSolver();
	World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);
	World->setGravity(btVector3(0.0, -9.81, 0.0));
	btContactSolverInfo &SolverInfo = World->getSolverInfo();
	SolverInfo.m_timeStep = TIMESTEP;
	//SolverInfo.m_solverMode = 0;
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
	if(BULLET_USE_STATIC_BOX) {

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

	if(BULLET_TEST_DETERMINISM) {
		int Iterations = 10.0 / TIMESTEP + 1;
		float Timer = 0;
		for(int i = 0; i < Iterations; i++) {
			Timer += TIMESTEP;

			SphereBody->applyTorque(btVector3(1, 0, 0));
			if(i == (int)(3.0 / TIMESTEP + 1))
				SphereBody->applyCentralImpulse(btVector3(0, 5, 0));
			World->stepSimulation(TIMESTEP, 0, 0);

			const btTransform &Transform = SphereBody->getCenterOfMassTransform();
			const btVector3 &Position = Transform.getOrigin();
			std::cout << std::setprecision(16) << "t=" << Timer << " " << Position[0] << " " << Position[1] << " " << Position[2] << std::endl;
		}
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
}

static void TestCalculations() {
	double Position[3] = { 1.00001, 1.00002, 1.00003 };
	for(int i = 0; i < 1000; i++) {
		Position[0] *= Position[1];
		Position[1] /= sqrt(pow(Position[2], 2));
		Position[2] += 0.00001 + Position[0];
	}

	std::cout << "TestCalculations" << std::endl;
	std::cout << std::setprecision(16) << Position[2] << std::endl;
}

int main(int ArgumentCount, char **Arguments) {

	//TestBullet();
	TestODE();
	TestCalculations();

	return 0;
}
