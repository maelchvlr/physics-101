#include "Scene.h"
#include "../Contact.h"
#include "../Shape.h"
#include "../Intersections.h"
#include "../Broadphase.h"
#include "../Utils.h"

Scene::~Scene() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();
}

void Scene::Reset() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();

	Initialize();
}

void Scene::Initialize() 
{
	float incrementalAngle = 0;
	float radiusArena = 10;
	float gap = 12;
	float n_balls = 40;

	Body ball_arena;
	for (int i = 0; i < n_balls; i++)
	{
		ball_arena.position = Vec3(cos(incrementalAngle) * radiusArena * gap, sin(incrementalAngle) * radiusArena * gap, 0);
		ball_arena.orientation = Quat(0, 0, 0, 1);
		ball_arena.shape = new ShapeSphere(radiusArena);
		ball_arena.inverseMass = 0.00f;
		ball_arena.elasticity = 0.5f;
		ball_arena.friction = 0.05f;
		ball_arena.linearVelocity = Vec3(0, 0, 0);
		incrementalAngle += 2 * 3.14159265 / n_balls;
		bodies.push_back(ball_arena);
	}

	float radius = 10000.0f;

	Body earth;
	earth.position = Vec3(0, 0, -radius);
	earth.orientation = Quat(0, 0, 0, 1);
	earth.shape = new ShapeSphere(radius);
	earth.inverseMass = 0.0f;
	earth.elasticity = 0.5f;
	earth.friction = 0.5f;
	bodies.push_back(earth);


	cochonnet.position = Vec3(0, 0, 10);
	cochonnet.orientation = Quat(0, 0, 0, 1);
	cochonnet.shape = new ShapeSphere(0.5);
	cochonnet.inverseMass = 0.01f;
	cochonnet.elasticity = 0.5f;
	cochonnet.friction = 0.5f;
	bodies.push_back(cochonnet);
	position_cochon = bodies.size() - 1;
	position_boule = bodies.size();

	Body body;
	for (int i = 0; i < 3; i++)
	{
		body.position = Vec3(45, 0, 10);
		body.orientation = Quat(0, 0, 0, 1);
		body.shape = new ShapeSphere(1);
		body.inverseMass = 0.001f;
		body.elasticity = 0.5f;
		body.friction = 0.5f;
	
		bodies.push_back(body);
	}
	
}

void Scene::Update(const float dt_sec)
{
	// Gravity
	for (int i = 0; i < bodies.size(); ++i)
	{
		Body& body = bodies[i];
		float mass = 1.0f / body.inverseMass;
		// Gravity needs to be an impulse I
		// I == dp, so F == dp/dt <=> dp = F * dt
		// <=> I = F * dt <=> I = m * g * dt
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * 50 * dt_sec;
		body.ApplyImpulseLinear(impulseGravity);

		float length = body.linearVelocity.GetLengthSqr();

		if (length > 0.5f)
		{
			body.linearVelocity *= 0.99f;
			body.angularVelocity *= 0.98f;
		}
		else
		{
			body.linearVelocity = Vec3(0, 0, 0);
			body.angularVelocity = Vec3(0, 0, 0);
		}
	}
	// Broadphase
	std::vector<CollisionPair> collisionPairs;
	BroadPhase(bodies.data(), bodies.size(), collisionPairs, dt_sec);
	// Collision checks (Narrow phase)
	int numContacts = 0;
	const int maxContacts = bodies.size() * bodies.size();
	Contact* contacts = (Contact*)_malloca(sizeof(Contact) * maxContacts);

	for (int i = 0; i < collisionPairs.size(); ++i)
	{
		const CollisionPair& pair = collisionPairs[i];
		Body& bodyA = bodies[pair.a];
		Body& bodyB = bodies[pair.b];
		if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f)
			continue;
		Contact contact;
		if (Intersections::Intersect(bodyA, bodyB, dt_sec, contact))
		{
			contacts[numContacts] = contact;
			++numContacts;
		}
	}
	// Sort times of impact
	if (numContacts > 1) {
		qsort(contacts, numContacts, sizeof(Contact),
			Contact::CompareContact);
	}
	// Contact resolve in order
	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; ++i)
	{
		Contact& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;
		Body* bodyA = contact.a;
		Body* bodyB = contact.b;
		// Skip body par with infinite mass
		if (bodyA->inverseMass == 0.0f && bodyB->inverseMass == 0.0f)
			continue;
		// Position update
		for (int j = 0; j < bodies.size(); ++j) {
			bodies[j].Update(dt);
		}
		Contact::ResolveContact(contact);
		accumulatedTime += dt;
	}
	// Other physics behavirous, outside collisions.
	// Update the positions for the rest of this frame's time.
	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < bodies.size(); ++i) {
			bodies[i].Update(timeRemaining);
		}
	}
}

void Scene::spawnCochonnet(Vec3 position, Vec3 rotation)
{
	if (!cochonnetThrown)
	{
		bodies[position_cochon].position = position;
		bodies[position_cochon].linearVelocity = rotation * 10;
		cochonnetThrown = true;
	}
	else
	{
		printf("cochonnet deja lance \n");
	}
}

void Scene::lanceBoule(Vec3 position, Vec3 rotation)
{
	if (compteur_boule < 3)
	{
		
		bodies[position_boule].position = position;
		bodies[position_boule].linearVelocity = rotation * 10;
		position_boule += compteur_boule;
		compteur_boule++;
	}
	else
	{
		printf("toutes les boules ont ete lancees \n");
	}
}

void Scene::resultat()
{

	int numBoule = 333;
	float mini = 999.f;
	for (int i = 0; i < 3; i++)
	{
		if ((bodies[position_cochon].position.GetLengthSqr() - bodies[position_cochon + 1 + i].position.GetLengthSqr()) < mini)
		{
			mini = bodies[position_cochon].position.GetLengthSqr() - bodies[position_cochon + 1 + i].position.GetLengthSqr();
			numBoule = i + 1;
		}
	}
	std::cout << "boule n°" << numBoule << "est la plus proche \n";
}




