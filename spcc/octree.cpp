#include "octree.h"
#include <string.h>
#include <sstream>


Octree::Octree(){
	for(int i = 0; i<8; i++){
		child[i] = NULL;
	}
}

Octree::~Octree(){
	for(size_t i=0; i<8; i++){
		delete child[i];
	}
}


void Octree::clearOctree(){
	points.clear();
	isLeaf = false;
	collidingPointsFromCheck.clear();
}

void Octree::printPoints(){
	for(size_t i=0; i<points.size(); ++i){
		std::cout << points[i] << std::endl;
	}
}

void Octree::printOctree(){
	if(isLeaf){
		printPoints();
		return;
	}

	for(int i = 0; i < 8; i++){
		if(child[i]->points.empty()){
			return; // nothing to print
		} else {
			child[i]->printOctree();
		}
	}
}

void Octree::traverseAndCheck(TransformedConvex* obj){
	collidingPointsFromCheck.clear();


	// box is at -1, 0, 0 -> point is at -1, 0.1, 0.1 and child 6 contains this point and box
	ccdw::vec3* pc = NULL;
	TransformedConvex* box = transform(new Box(ccdw::vec3(radius)), Transform3(center));
	Report report;

	// ccdw::vec3 objCenter, boxCenter;
	// obj->center(objCenter);
	// box->center(boxCenter);



	// first check if the object hits the bounding box of this octree
	if(checker.query(qtype, &report, box, obj, 0)){
		// the object hits the dialated bounding box of this octree

		// if you are a leaf, check if the obj hits any of the points
		if(isLeaf){
			for(size_t i=0; i<points.size(); i++){
				if(obj->contains(points[i], pc)){
					// this object collides with this point. make a report

					// std::cout << "COLLIDES WITH POINT: " << points[i] << std::endl;
					
					collidingPointsFromCheck.push_back(points[i]);

				}
			}
		} else {

			//otherwise traverse into each child and check if the object hits the leaf
			delete box->child;
			delete box;

			for(int i=0; i<8; i++){
				// check if the child is defined
				if(child[i] != NULL){
					child[i]->traverseAndCheck(obj);

					// if the child has any colliding points, copy it up
					for(size_t j=0; j<child[i]->collidingPointsFromCheck.size(); j++){
						collidingPointsFromCheck.push_back(child[i]->collidingPointsFromCheck[j]);
					}

				} 

			}
			return;  
		}
	} 

	// otherwise the object doesn't hit this bounding box and it doesn't matter
	delete box->child;
	delete box;

}

// dmin = 0 here. No dialation around point cloud. 
bool Octree::checkForCollisions(TransformedConvex* obj, size_t objIndex, std::vector<CollidingObjects> &spccReportMasterList){

	traverseAndCheck(obj);

	CollidingObjects spccReport;

	if(collidingPointsFromCheck.size() > 0){
		CollidingObjects spccReport;
		spccReport.objectIndex = objIndex;
		spccReport.collidingPoints = collidingPointsFromCheck;
		
		spccReportMasterList.push_back(spccReport);

		return true;
	}

	// push back an empty report so that the report indicies match up with cgeom/object array indices
	spccReportMasterList.push_back(spccReport);

	return false;

}

// set dmin when calling checkForCollisions
bool Octree::checkForCollisions(TransformedConvex* obj, size_t objIndex, std::vector<CollidingObjects> &spccReportMasterList, ccd_real_t dmin){

	Convex* dilatedObj = 0;

	if(dmin){
		dilatedObj = dilate(obj, dmin);
		obj = (TransformedConvex*) dilatedObj;
	}

	traverseAndCheck(obj);

	CollidingObjects spccReport;

	if(collidingPointsFromCheck.size() > 0){
		CollidingObjects spccReport;
		spccReport.objectIndex = objIndex;
		spccReport.collidingPoints = collidingPointsFromCheck;
		
		spccReportMasterList.push_back(spccReport);

		return true;
	}

	// push back an empty report so that the report indicies match up with cgeom/object array indices
	spccReportMasterList.push_back(spccReport);

	return false;

}

bool Octree::buildOctree(const std::vector<ccdw::vec3>& incomingPoints,
													 int threshold,
													 int maxDepth,
													 Bounds &b,
													 int currDepth){

	if(height == 0){
		center = b.center;
		radius = b.radius;
	}

	// check if you are a leaf
	// you are a leaf if 1. num points <= threshold
	//                   2. currDepth >= maxDepth

	if(incomingPoints.size() <= (unsigned int) threshold || currDepth >= maxDepth){
		points = incomingPoints;
		isLeaf = true;
		return true;
	}

	isLeaf = false;

	// classify each point to a child node

	std::vector<ccdw::vec3> cubePointList[8];

	// std::vector<ccdw::vec3> cube0; // +x, +y, +z
	// std::vector<ccdw::vec3> cube1; // +x, +y, -z
	// std::vector<ccdw::vec3> cube2; // +x, -y, +z 
	// std::vector<ccdw::vec3> cube3; // +x, -y, -z
	// std::vector<ccdw::vec3> cube4; // -x, +y, +z
	// std::vector<ccdw::vec3> cube5; // -x, +y, -z
	// std::vector<ccdw::vec3> cube6; // -x, -y, +z
	// std::vector<ccdw::vec3> cube7; // -x, -y, -z

	ccd_real_t centerX = b.center[0];
	ccd_real_t centerY = b.center[1];
	ccd_real_t centerZ = b.center[2];

	for(size_t i=0; i<incomingPoints.size(); ++i){

		// current point
		ccdw::vec3 p = incomingPoints[i];
		ccd_real_t pX = p[0];
		ccd_real_t pY = p[1];
		ccd_real_t pZ = p[2];

		if(pX >= centerX){ // +x
			if(pY >= centerY){ // +x, +y
				if(pZ >= centerZ){ // +x, +y, +z
					// point belongs in cube 0
					cubePointList[0].push_back(p);
				} else { // +x, +y, -z
					// point belongs to cube 1
					cubePointList[1].push_back(p);
				}
			} else { // +x, -y
				if(pZ >= centerZ){ // +x, -y, +z
					// point belongs in cube 2
					cubePointList[2].push_back(p);
				} else { // +x, -y, -z
					// point belongs to cube 3
					cubePointList[3].push_back(p);
				}
			}
		} else { // -x
			if(pY >= centerY){ // -x, +y
				if(pZ >= centerZ){ // -x, +y, +z
					// point belongs in cube 4
					cubePointList[4].push_back(p);
				} else { // -x, +y, -z
					// point belongs to cube 5
					cubePointList[5].push_back(p);
				}
			} else { // -x, -y
				if(pZ >= centerZ){ // +x, -y, +z
					// point belongs in cube 6
					cubePointList[6].push_back(p);
				} else { // -x, -y, -z
					// point belongs to cube 7
					cubePointList[7].push_back(p);
				}
			}
		}
	}


	// recursively create the 8 subTrees
	for(int i = 0; i < 8; i++){

		if(!cubePointList[i].size()){ // no points in this cube
			child[i] = NULL;
			continue;
		} 


		if(child[i] == NULL){
			child[i] = new Octree();
		} else {
			child[i]->clearOctree();
		}


		ccdw::vec3 offset = boundsOffsetTable[i] * b.radius * 0.5;
		Bounds newBounds;
		newBounds.radius = b.radius * 0.5;
		newBounds.center = b.center + offset;

		child[i]->radius = newBounds.radius;
		child[i]->center = newBounds.center;
		child[i]->height = currDepth + 1;
		child[i]->checker = checker;
		child[i]->qtype = qtype;

		child[i]->buildOctree(cubePointList[i], threshold, maxDepth, newBounds, currDepth + 1);

	}

	return true;
}


Bounds Octree::boundingBox(const std::vector<ccdw::vec3>& points){

	Bounds b;

	// Find bounding box
	ccd_real_t xlo= 10000; 
	ccd_real_t ylo = 10000;
	ccd_real_t zlo = 10000;
	ccd_real_t xhigh = -10000;
	ccd_real_t yhigh = -10000; 
	ccd_real_t zhigh = -10000; 

	ccd_real_t x, y, z;

	for (size_t i=0; i<points.size(); ++i) {

		x = points[i][0];
		y = points[i][1];
		z = points[i][2];

		if (x < xlo){
			xlo = x;
		} else if (x > xhigh){
			xhigh = x;
		}

		if (y < ylo){
			ylo = y;
		} else if (y > yhigh){
			yhigh = y;
		}

		if (z < zlo){
			zlo = z;
		} else if (z > zhigh){
			zhigh = z;
		}

	}

	ccdw::vec3 min = ccdw::vec3(xlo, ylo, zlo);
	ccdw::vec3 max = ccdw::vec3(xhigh, yhigh, zhigh);

	// The radius (dimensions in each direction)
	ccdw::vec3 radius = max - min; // length of the edge of square

	std::cout << "r: " << radius[0] << " " << radius[1] << " " << radius[2] << std::endl;

	b.center = min + radius * 0.5;

	b.radius = radius[0]; // first set to x
	if (b.radius < radius[1]) b.radius = radius[1];
	if (b.radius < radius[2]) b.radius = radius[2];

	b.radius = b.radius; // fudge factor
	return b;
}
