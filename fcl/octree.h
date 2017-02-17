#ifndef _OCTREE_H_
#define _OCTREE_H_

#include "CCDWrapper.h"

using namespace ccdw;


typedef struct {
  vec3 center;
  ccd_real_t radius;
} Bounds;


class Octree {
public:

	Octree();
	virtual ~Octree();

	// Accessors
	virtual std::vector<vec3> points() = 0;

	// tree implementation
	virtual bool buildTree(std::vector<vec3> points,
						   int threshold,
						   int maxDepth,
						   Bounds &b,
						   int currentDepth) = 0;

	virtual Bounds boundingBox(std::vector<vec3> points) = 0;
	virtual bool traverse();


	Octree* child[8];
	std::vector<vec3> points;
	vec3 center;
	ccd_real_t radius;
};
