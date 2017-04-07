#ifndef _OCTREE_H_
#define _OCTREE_H_

#include "CCDWrapper.h"

using namespace ccdw;

static const ccdw::vec3 boundsOffsetTable[8] = {
  ccdw::vec3(+0.5, +0.5, +0.5),
  ccdw::vec3(+0.5, +0.5, -0.5),
  ccdw::vec3(+0.5, -0.5, +0.5),
  ccdw::vec3(+0.5, -0.5, -0.5),
  ccdw::vec3(-0.5, +0.5, +0.5),
  ccdw::vec3(-0.5, +0.5, -0.5),
  ccdw::vec3(-0.5, -0.5, +0.5),
  ccdw::vec3(-0.5, -0.5, -0.5),
};


typedef struct {
  ccdw::vec3 center;
  ccd_real_t radius;
} Bounds;

typedef struct{
  size_t objectIndex;
  std::vector<ccdw::vec3> collidingPoints;
} CollidingObjects;


class Octree {
public:

	Octree();
	virtual ~Octree();

	void clearOctree();
	void printPoints();
	void printOctree();
	void traverseAndCheck(TransformedConvex* obj);
	bool checkForCollisions(TransformedConvex* obj, size_t objIndex, std::vector<CollidingObjects> &spccReportMasterList);
	bool buildOctree(std::vector<ccdw::vec3> incomingPoints, int threshold, int maxDepth, Bounds &b, int currDepth);
	Bounds boundingBox(std::vector<ccdw::vec3> points);


	Octree* child[8];
	std::vector<ccdw::vec3> points;
	ccdw::vec3 center;
	ccd_real_t radius;
	int height;
	bool isLeaf;
	std::vector<ccdw::vec3> collidingPointsFromCheck;;
	Checker checker;
	QueryType qtype;

};

#endif
