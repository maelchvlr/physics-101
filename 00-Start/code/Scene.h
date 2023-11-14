#pragma once
#include <vector>

#include "../Body.h"

class Scene 
{
public:
	Scene() { bodies.reserve( 128 ); }
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );	
	void spawnCochonnet(Vec3 position, Vec3 rotation);
	void lanceBoule(Vec3 position, Vec3 rotation);
	void resultat();

	std::vector<Body> bodies;
	Body earth;

	bool isCochonnetPlayed = false;
	Body cochonnet;
	int position_cochon;
	int position_boule;
	int compteur_boule = 0;
	bool cochonnetThrown = false;
	bool allBouleThrown = false;
	std::vector<Body> petanque_balls;
};

