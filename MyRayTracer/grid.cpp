#include <iostream>
#include <string>
#include <fstream>
#include <IL/il.h>
#include "maths.h"
#include "grid.h"

Grid::Grid()
{
}

Grid::Grid(vector<Object*> objs) {
	objects = objs;
	bbox = new AABB();
	bbox->min = find_min_bounds();
	bbox->max = find_max_bounds();

	dim = bbox->max - bbox->min;

	float s = pow(objects.size()/(dim.x * dim.y * dim.z), 1.f / 3.f);
	nx = trunc(m * dim.x * s) + 1;
	ny = trunc(m * dim.y * s) + 1;
	nz = trunc(m * dim.z * s) + 1;
	

	int total_cells = nx * ny * nz;



	for (int i = 0; i < total_cells; i++) {
		Cell cell = Cell();
		cell.objects = {};
		cells.push_back(cell);
	}
	
	for (int i = 0; i < objects.size(); i++) {
		Vector objBBmin = objects[i]->GetBoundingBox().min;
		Vector objBBmax = objects[i]->GetBoundingBox().max;

		/* Compute indices of both cells that contain min and max coord of obj bbox */
		int ixmin = clamp(((objBBmin.x - bbox->min.x) * nx / dim.x), 0, nx - 1);
		int iymin = clamp(((objBBmin.y - bbox->min.y) * ny / dim.y), 0, ny - 1);
		int izmin = clamp(((objBBmin.z - bbox->min.z) * nz / dim.z), 0, nz - 1);
		int ixmax = clamp(((objBBmax.x - bbox->min.x) * nx / dim.x), 0, nx - 1);
		int iymax = clamp(((objBBmax.y - bbox->min.y) * ny / dim.y), 0, ny - 1);
		int izmax = clamp(((objBBmax.z - bbox->min.z) * nz / dim.z), 0, nz - 1);

		/* insert obj to the overlaped cells */
		for (int iz = izmin; iz <= izmax; iz++) {
			for (int iy = iymin; iy <= iymax; iy++) {
				for (int ix = ixmin; ix <= ixmax; ix++) {
					int index = ix + nx * iy + nx * ny * iz;
					cells[index].add(objects[i]);
				}
			}
		}
	}
}


Vector Grid::find_min_bounds(void) {

	Vector min = Vector(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

	for (int i = 0; i < objects.size(); i++) {
		Object* object = objects[i];

		Vector obj_min = object->GetBoundingBox().min;
		if (obj_min.x < min.x)  min.x = obj_min.x - 0.0001f;
		if (obj_min.y < min.y)  min.y = obj_min.y - 0.0001f;
		if (obj_min.z < min.z)  min.z = obj_min.z - 0.0001f;
	}

	return (min);
}


Vector Grid::find_max_bounds(void) {

	Vector max = Vector(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min());

	for (int i = 0; i < objects.size(); i++) {
		Object* object = objects[i];

		Vector obj_max = object->GetBoundingBox().max;
		if (obj_max.x > max.x)  max.x = obj_max.x + 0.0001f;
		if (obj_max.y > max.y)  max.y = obj_max.y + 0.0001f;
		if (obj_max.z > max.z)  max.z = obj_max.z + 0.0001f;
	}

	return (max);
}

Object* Grid::rayIntersection(Ray & ray, Vector* intersectionPoint, Vector* normalIntersection) {
	float t;
	Vector min;
	Vector max;
	if (!bbox->intercepts(ray, t, max, min)) return nullptr;

	/* Calculate Starting Cell */
	Vector cellPos;
	if (bbox->isInside(ray.origin)) {
		/*if the ray starts inside the Grid find the cell that contain the ray origin*/
		cellPos.x = clamp((int)((ray.origin.x - bbox->min.x) * nx / dim.x), 0, int(nx - 1));
		cellPos.y = clamp((int)((ray.origin.y - bbox->min.y) * ny / dim.y), 0, int(ny - 1));
		cellPos.z = clamp((int)((ray.origin.z - bbox->min.z) * nz / dim.z), 0, int(nz - 1));
	}
	else {
		/*find the cell where the ray hits the Grid from the outside*/
		Vector hitPoint = ray.origin + ray.direction * t;
		cellPos.x = clamp((int)((hitPoint.x - bbox->min.x) * nx / dim.x), 0, int(nx - 1));
		cellPos.y = clamp((int)((hitPoint.y - bbox->min.y) * ny / dim.y), 0, int(ny - 1));
		cellPos.z = clamp((int)((hitPoint.z - bbox->min.z) * nz / dim.z), 0, int(nz - 1));
	}
	//Setup Traversing
	Vector dt = Vector((max.x - min.x) / nx, (max.y - min.y) / ny, (max.z - min.z) / nz);

	Vector tnext;
	Vector step;
	Vector stop;

	if (ray.direction.x > 0) {
		tnext.x = min.x + (cellPos.x + 1) * dt.x;
		step.x = 1;
		stop.x = nx;
	}
	else {
		tnext.x = min.x + (nx - cellPos.x) * dt.x;
		step.x = -1;
		stop.x = -1;
	}
	if (ray.direction.x == 0) {
		tnext.x = +INFINITY;
	}

	if (ray.direction.y > 0) {
		tnext.y = min.y + (cellPos.y + 1) * dt.y;
		step.y = 1;
		stop.y = ny;
	}
	else {
		tnext.y = min.y + (ny - cellPos.y) * dt.y;
		step.y = -1;
		stop.y = -1;
	}
	if (ray.direction.y == 0) {
		tnext.y = +INFINITY;
	}

	if (ray.direction.z > 0) {
		tnext.z = min.z + (cellPos.z + 1) * dt.z;
		step.z = 1;
		stop.z = nz;
	}
	else {
		tnext.z = min.z + (nz - cellPos.z) * dt.z;
		step.z = -1;
		stop.z = -1;
	}
	if (ray.direction.z == 0) {
		tnext.z = +INFINITY;
	}

	return gripTraversalLoop(cellPos, ray, tnext, dt, step, stop, intersectionPoint, normalIntersection);
}

bool Grid::Traverse(Ray& ray) {
	float t;
	Vector min;
	Vector max;

	bbox->intercepts(ray, t, max, min);

	/* Calculate Starting Cell */
	Vector cellPos;
	if (bbox->isInside(ray.origin)) {
		/*if the ray starts inside the Grid find the cell that contain the ray origin*/
		cellPos.x = clamp((int)((ray.origin.x - bbox->min.x) * nx / dim.x), 0, int(nx - 1));
		cellPos.y = clamp((int)((ray.origin.y - bbox->min.y) * ny / dim.y), 0, int(ny - 1));
		cellPos.z = clamp((int)((ray.origin.z - bbox->min.z) * nz / dim.z), 0, int(nz - 1));
	}
	else {
		/*find the cell where the ray hits the Grid from the outside*/
		Vector hitPoint = ray.origin + ray.direction * t;
		cellPos.x = clamp((int)((hitPoint.x - bbox->min.x) * nx / dim.x), 0, int(nx - 1));
		cellPos.y = clamp((int)((hitPoint.y - bbox->min.y) * ny / dim.y), 0, int(ny - 1));
		cellPos.z = clamp((int)((hitPoint.z - bbox->min.z) * nz / dim.z), 0, int(nz - 1));
	}
	
	//Setup Traversing
	Vector dt = Vector((max.x - min.x) / nx, (max.y - min.y) / ny, (max.z - min.z) / nz);

	Vector tnext;
	Vector step;
	Vector stop;

	if (ray.direction.x > 0) {
		tnext.x = min.x + (cellPos.x + 1) * dt.x;
		step.x = 1;
		stop.x = nx;
	}
	else {
		tnext.x = min.x + (nx - cellPos.x) * dt.x;
		step.x = -1;
		stop.x = -1;
	}
	if (ray.direction.x == 0) {
		tnext.x = +INFINITY;
	}

	if (ray.direction.y > 0) {
		tnext.y = min.y + (cellPos.y + 1) * dt.y;
		step.y = 1;
		stop.y = ny;
	}
	else {
		tnext.y = min.y + (ny - cellPos.y) * dt.y;
		step.y = -1;
		stop.y = -1;
	}
	if (ray.direction.y == 0) {
		tnext.y = +INFINITY;
	}

	if (ray.direction.z > 0) {
		tnext.z = min.z + (cellPos.z + 1) * dt.z;
		step.z = 1;
		stop.z = nz;
	}
	else {
		tnext.z = min.z + (nz - cellPos.z) * dt.z;
		step.z = -1;
		stop.z = -1;
	}
	if (ray.direction.z == 0) {
		tnext.z = +INFINITY;
	}

	while (true) {
		std::vector<Object*> objects = cells[cellPos.x + nx * cellPos.y + nx * ny * cellPos.z].getObjects();
		float t;
		for (int i = 0; i < objects.size(); i++) {
			if (objects[i]->intercepts(ray, t))
				return true;
		}
		if (tnext.x < tnext.y && tnext.x < tnext.z) {
			tnext.x += dt.x;
			cellPos.x += step.x;
			if (cellPos.x == stop.x) return false;
		}
		else if (tnext.y < tnext.z) {
			tnext.y += dt.y;
			cellPos.y += step.y;
			if (cellPos.y == stop.y) return false;
		}
		else {
			tnext.z += dt.z;
			cellPos.z += step.z;
			if (cellPos.z == stop.z) return false;
		}

	}
}

Object* Grid::gripTraversalLoop(Vector cellPos, Ray ray, Vector tnext, Vector dt, Vector step, Vector stop, Vector* intersectionPoint, Vector* normalIntersection) {
	while (true) {

		std::vector<Object*> objects = cells[cellPos.x + nx * cellPos.y + nx * ny * cellPos.z].getObjects();

		Object* hitObject = nullptr;

		float auxDistance;
		float minDistance = INFINITY;
		for (int i = 0; i < objects.size(); i++) {
			if (objects[i]->intercepts(ray, auxDistance)) {
				if (auxDistance < minDistance) {
					hitObject = objects[i];
					minDistance = auxDistance;
					*intersectionPoint = ray.origin + ray.direction * minDistance;
					*normalIntersection = objects[i]->getNormal(*intersectionPoint).normalize();
				}
			}
		}

		if (tnext.x < tnext.y && tnext.x < tnext.z) {
			if (hitObject && minDistance < tnext.x) return hitObject;
			tnext.x += dt.x;
			cellPos.x += step.x;
			if (cellPos.x == stop.x) return nullptr;
		}
		else if (tnext.y < tnext.z) {
			if (hitObject && minDistance < tnext.y) return hitObject;
			tnext.y += dt.y;
			cellPos.y += step.y;
			if (cellPos.y == stop.y) return nullptr;
		}
		else {
			if (hitObject && minDistance < tnext.z) return hitObject;
			tnext.z += dt.z;
			cellPos.z += step.z;
			if (cellPos.z == stop.z) return nullptr;
		}
	}
}