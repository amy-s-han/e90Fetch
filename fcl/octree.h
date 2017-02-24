#ifndef _OCTREE_H_
#define _OCTREE_H_

#include "CCDWrapper.h"

using namespace ccdw;

static const vec3 boundsOffsetTable[8] = {
  vec3(+0.5, +0.5, +0.5),
  vec3(+0.5, +0.5, -0.5),
  vec3(+0.5, -0.5, +0.5),
  vec3(+0.5, -0.5, -0.5),
  vec3(-0.5, +0.5, +0.5),
  vec3(-0.5, +0.5, -0.5),
  vec3(-0.5, -0.5, +0.5),
  vec3(-0.5, -0.5, -0.5),
};


typedef struct {
  vec3 center;
  ccd_real_t radius;
} Bounds;

typedef struct{
  size_t objectIndex;
  std::vector<vec3> collidingPoints;
} CollidingObjects;


class Octree {
public:

	Octree();
	virtual ~Octree();

	void clearOctree();
	void printPoints();
	void printOctree();
	void traverseAndCheck(TransformedConvex* obj, size_t objIndex);
	bool checkForCollisions(TransformedConvex* obj, size_t objIndex, std::vector<CollidingObjects> &fclReportMasterList);
	bool buildOctree(std::vector<vec3> incomingPoints, int threshold, int maxDepth, Bounds &b, int currDepth);
	Bounds boundingBox(std::vector<vec3> points);





	Octree* child[8];
	std::vector<vec3> points;
	vec3 center;
	ccd_real_t radius;
	int height;
	bool isLeaf;
	std::vector<CollidingObjects> fclReport;
	Checker checker;
	QueryType qtype;

};

#endif
