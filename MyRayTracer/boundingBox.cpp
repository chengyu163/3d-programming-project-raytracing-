#ifndef AABB_H
#define AABB_H

#include "vector.h"
#include "boundingBox.h"

//-------------------------------------------------------------------- - default constructor
AABB::AABB(void) 
{
	min = Vector(-1.0f, -1.0f, -1.0f);
	max = Vector(1.0f, 1.0f, 1.0f);
}

// --------------------------------------------------------------------- constructor
AABB::AABB(const Vector& v0, const Vector& v1)
{
	min = v0; max = v1;
}

// --------------------------------------------------------------------- copy constructor
AABB::AABB(const AABB& bbox) 
{
	min = bbox.min; max = bbox.max;
}

// --------------------------------------------------------------------- assignment operator
AABB AABB::operator= (const AABB& rhs) {
	if (this == &rhs)
		return (*this);
	min = rhs.min;
	max = rhs.max;
	return (*this);
}

// --------------------------------------------------------------------- destructor
AABB::~AABB() {}

// --------------------------------------------------------------------- inside
// used to test if a ray starts inside a grid

bool AABB::isInside(const Vector& p) 
{
	return ((p.x > min.x && p.x < max.x) && (p.y > min.y && p.y < max.y) && (p.z > min.z && p.z < max.z));
}

bool AABB::intercepts(const Ray& ray, float& t, Vector& max, Vector& min)
{
	float tDist = INFINITY;
	float tProx = -INFINITY;
	float tx_max, ty_max, tz_max, tx_min, ty_min, tz_min;

	float a = 1.0f / ray.direction.x;
	if (a >= 0) {
		tx_min = (this->min.x - ray.origin.x) * a;
		tx_max = (this->max.x - ray.origin.x) * a;

	}
	else {
		tx_max = (this->min.x - ray.origin.x) * a;
		tx_min = (this->max.x - ray.origin.x) * a;

	}
	
	float b = 1.0f / ray.direction.y;
	if (b >= 0) {
		ty_min = (this->min.y - ray.origin.y) * b;
		ty_max = (this->max.y - ray.origin.y) * b;
	}
	else {
		ty_max = (this->min.y - ray.origin.y) * b;
		ty_min = (this->max.y - ray.origin.y) * b;
	}

	float c = 1.0f / ray.direction.z;
	if (c >= 0) {
		tz_min = (this->min.z - ray.origin.z) * c;
		tz_max = (this->max.z - ray.origin.z) * c;
	}
	else {
		tz_max = (this->min.z - ray.origin.z) * c;
		tz_min = (this->max.z - ray.origin.z) * c;
	}
	if (tx_min > ty_min)
		tProx = tx_min;
	else
		tProx = ty_min;
	if (tz_min > tProx)
		tProx = tz_min;
	if (tx_max < ty_max)
		tDist = tx_max;
	else
		tDist = ty_max;
	if (tz_max < tDist)
		tDist = tz_max;
	if (tProx > tDist || tDist < 0)
		return false;
	t = tProx;
	max.x = tx_max;
	max.y = ty_max;
	max.z = tz_max;
	min.x = tx_min;
	min.y = ty_min;
	min.z = tz_min;
	return true;

}
#endif