#ifndef GRID_H
#define GRID_H

#include <vector>
#include <cmath>
#include "scene.h"


using namespace std;

class Grid
{
	struct Cell
	{
		vector<Object*> objects;

		void add(Object* object) {
			objects.push_back(object);
		}

		vector<Object*> getObjects() {
			return objects;
		}
	};
	
public:
	Grid(void);
	Grid(vector<Object*> objects);
	//~Grid(void);

	int getNumObjects();
	void addObject(Object* o);
	Object* getObject(unsigned int index);

	void Build(void);   // set up grid cells

	bool Traverse(Ray& ray, Object **hitobject, Vector& hitpoint);  //(const Ray& ray, double& tmin, ShadeRec& sr)
	bool Traverse(Ray& ray);  //Traverse for shadow ray
	Object* rayIntersection(Ray& ray, Vector* intersectionPoint, Vector* normalIntersection);

private:
	vector<Object *> objects;
	vector<Cell> cells;

	int nx, ny, nz; // number of cells in the x, y, and z directions
	float m = 2.0f; // factor that allows to vary the number of cells

	Vector find_min_bounds(void);
	Vector find_max_bounds(void);


	Object* gripTraversalLoop(Vector cellPos, Ray ray, Vector tnext, Vector dt, Vector step, Vector stop, Vector* intersectionPoint, Vector* normalIntersection);
	//Setup function for Grid traversal
	bool Init_Traverse(Ray& ray, int& ix, int& iy, int& iz, double& dtx, double& dty, double& dtz, double& tx_next, double& ty_next, double& tz_next, 
		int& ix_step, int& iy_step, int& iz_step, int& ix_stop, int& iy_stop, int& iz_stop);

	AABB *bbox;
	Vector dim;
};
#endif
