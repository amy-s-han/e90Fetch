#include "octree.h"
#include <string.h>
#include <sstream>



Octree::Octree(){
	memset(child, 0, sizeof(child));
}

Octree::~Octree(){
	//delete stuff?
}

std::vector<vec3> Octree::points(){
	return points
}

bool Octree::buildTree(std::vector<vec3> points,
						   int threshold,
						   int maxDepth,
						   Bounds &b,
						   int currentDepth){


}

Bounds Octree::boundingBox(std::vector<vec3> points){

	Bounds b;

	// Find bounding box
    ccd_real_t xlo= 0; 
    ccd_real_t ylo = 0;
    ccd_real_t xhigh = 0;
    ccd_real_t yhigh = 0; 
    ccd_real_t zlo = 0;
    ccd_real_t zhigh = 0; 

    ccd_real_t x, y, z;

    for (size_t i=0; i<points.size(); ++i) {
      std::cout << "Point " << i << ": ";
      std::cout << points[i] << std::endl;

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

    vec3 min = vec3(xlo, ylo, zlo);
    vec3 max = vec3(xhigh, yhigh, zhigh);

    // The radius (dimensions in each direction)
    vec3 radius = max - min;

    b.center = min + radius * 0.5;

    b.radius = radius[0]; // first set to x
    if (b.radius < radius[1]) b.radius = radius[1];
    if (b.radius < radius[2]) b.radius = radius[2];

    b.radius = b.radius * 1.05; // fudge factor

    std::cout << "Bounding vars, max: " << max << " and min: " << min << std::endl;
    std::cout << "center is: " << b.center << std::endl;
    std::cout << "box radius: " << b.radius << std::endl;

    return b;
}


bool Octree::traverse(){
	//not sure what to do here yet

}