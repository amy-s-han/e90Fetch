#include "CCDWrapper.h"
#include <mzcommon/glstuff.h>
#include <mzcommon/MzGlutApp.h>
#include <mzcommon/mersenne.h>
#include <mzcommon/TimeUtil.h>
#include <sstream>

using namespace ccdw;

enum { ncolors = 6 };

static const vec3 ccolors[ncolors] = {
  vec3(1.0, 0.0, 0.0),
  vec3(1.0, 1.0, 0.0),
  vec3(0.0, 1.0, 0.0),
  vec3(0.0, 1.0, 1.0),
  vec3(0.0, 0.0, 1.0),
  vec3(1.0, 0.0, 1.0),
};

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
  Octree* child[8];
  std::vector<vec3> points;
  vec3 center;
  ccd_real_t radius;
  int height;
  bool isLeaf;
  std::vector<CollidingObjects> fclReport;
  Checker checker;
  QueryType qtype;

  Octree(){
    for(int i = 0; i<8; i++){
      child[i] = NULL;
    }
  }

  void clearOctree(){
    points.clear();
    isLeaf = false;
    fclReport.clear();
  }

  void printPoints(){
    for(size_t i=0; i<points.size(); ++i){
      std::cout << points[i] << std::endl;
    }
  }

  void printOctree(){
    if(isLeaf){
      std::cout << "Leaf Node of height " << height << " and points: " << std::endl;
      printPoints();
      return;
    }

    for(int i = 0; i < 8; i++){
      std::cout << "Printing child " << i << std::endl;
      if(child[i]->points.empty()){
        return; // nothing to print
      } else {
        child[i]->printOctree();
      }
    }
  }

  void traverseAndCheck(TransformedConvex* obj, size_t objIndex){
    std::cout << "hereeeee and center: " << center << "and radius: " << radius << std::endl << std::flush;
    std::cout << "DEPTH: " << height << std::endl;

    // box is at -1, 0, 0 -> point is at -1, 0.1, 0.1 and child 6 contains this point and box
    vec3* pc = NULL;
    TransformedConvex* box = transform(new Box(vec3(radius)), Transform3(center));
    Report report;

    // first check if the object hits the bounding box of this octree
    if(checker.query(qtype, &report, box, obj, 0)){
      // the object hits the bounding box of this octree
      std::cout << "OBJECT HITS THIS CHILD" << std::endl;

      // if you are a leaf, check if the obj hits any of the points
      if(isLeaf){
        std::cout << "I'M A LEAF!!!!!!!!!" << std::endl;
        for(size_t i=0; i<points.size(); i++){
          if(obj->contains(points[i], pc)){
            // this object collides with this point. make a report
            std::cout << "THERE'S BEEN A COLLISION!!!!!!!!" << std::endl;

            // check if there is already a report for this object:
            bool reportExists = false;
            for(size_t j=0; j<fclReport.size(); j++){
              if(fclReport[j].objectIndex == objIndex){
                fclReport[j].collidingPoints.push_back(points[i]);
                reportExists = true;
                break;
              } 
            }

            if(!reportExists){ // make a report for this object   
              CollidingObjects c;
              c.objectIndex = objIndex;
              c.collidingPoints.push_back(points[i]);
              fclReport.push_back(c);

            }
          }
        }
      } else {

        //otherwise traverse into each child and check if the object hits the leaf
        delete box;

        for(int i=0; i<8; i++){
          // check if the child is defined
          if(child[i] != NULL){
            std::cout << "\n\ntraversing child: " << i << std::endl;
            child[i]->traverseAndCheck(obj, objIndex);

            // if the child made a collision report, copy it up
            for(size_t j=0; j<child[i]->fclReport.size(); j++){
              fclReport.push_back(child[i]->fclReport[j]);
            }
 
          } else {
            std::cout << "child " << i << " is null so skipping traversal" << std::endl;
          }

        }
        return;  
      }
    } 

    // otherwise the object doesn't hit this bounding box and it doesn't matter
    delete box;
    
  }

  bool checkForCollisions(TransformedConvex* obj, size_t objIndex, std::vector<CollidingObjects> &fclReportMasterList){

    traverseAndCheck(obj, objIndex);

    if(fclReport.size() > 0){
      for(size_t i=0; i<fclReport.size(); i++){
        fclReportMasterList.push_back(fclReport[i]);
        std::cout << "copying report: " << i << std::endl;
      }
      return true;
    }
    
    return false;

  }

  bool buildOctree(std::vector<vec3> incomingPoints,
               int threshold,
               int maxDepth,
               Bounds &b,
               int currDepth){

    // check if you are a leaf
    // you are a leaf if 1. num points <= threshold
    //                   2. currDepth >= maxDepth


    for(size_t i = 0; i < incomingPoints.size(); i++){
      // std::cout << "incomingPoints```" << incomingPoints[i] << std::endl;
      points.push_back(incomingPoints[i]);
    }


    // for(size_t i = 0; i < points.size(); i++){
    //   std::cout << "points```" << points[i] << std::endl;
    // }

    if(incomingPoints.size() <= (unsigned int) threshold || currDepth >= maxDepth){
      points = incomingPoints;
      isLeaf = true;
      return true;
    }

    isLeaf = false;

    // classify each point to a child node
    int cubePointCounts[8] = {0};
    
    std::vector<vec3> cube0; // +x, +y, +z
    std::vector<vec3> cube1; // +x, +y, -z
    std::vector<vec3> cube2; // +x, -y, +z
    std::vector<vec3> cube3; // +x, -y, -z
    std::vector<vec3> cube4; // -x, +y, +z
    std::vector<vec3> cube5; // -x, +y, -z
    std::vector<vec3> cube6; // -x, -y, +z
    std::vector<vec3> cube7; // -x, -y, -z

    ccd_real_t centerX = b.center[0];
    ccd_real_t centerY = b.center[1];
    ccd_real_t centerZ = b.center[2];
    
    for(size_t i=0; i<points.size(); ++i){

      // current point
      vec3 p = points[i];
      ccd_real_t pX = p[0];
      ccd_real_t pY = p[1];
      ccd_real_t pZ = p[2];

      if(pX >= centerX){ // +x
        if(pY >= centerY){ // +x, +y
          if(pZ >= centerZ){ // +x, +y, +z
            // point belongs in cube 0
            cube0.push_back(p);
            cubePointCounts[0] ++;
          } else { // +x, +y, -z
            // point belongs to cube 1
            cube1.push_back(p);
            cubePointCounts[1] ++; 
          }
        } else { // +x, -y
          if(pZ >= centerZ){ // +x, -y, +z
            // point belongs in cube 2
            cube2.push_back(p);
            cubePointCounts[2] ++;
          } else { // +x, -y, -z
            // point belongs to cube 3
            cube3.push_back(p);
            cubePointCounts[3] ++; 
          }
        }
      } else { // -x
        if(pY >= centerY){ // -x, +y
          if(pZ >= centerZ){ // -x, +y, +z
            // point belongs in cube 4
            cube4.push_back(p);
            cubePointCounts[4] ++;
          } else { // -x, +y, -z
            // point belongs to cube 5
            cube5.push_back(p);
            cubePointCounts[5] ++; 
          }
        } else { // -x, -y
          if(pZ >= centerZ){ // +x, -y, +z
            // point belongs in cube 6
            cube6.push_back(p);
            cubePointCounts[6] ++;
          } else { // -x, -y, -z
            // point belongs to cube 7
            cube7.push_back(p);
            cubePointCounts[7] ++; 
          }
        }
      }
    }
    std::vector<vec3> cubePointList[8] = {cube0, cube1, cube2, cube3, cube4, cube5, cube6, cube7};

    // recursively create the 8 subTrees
    for(int i = 0; i < 8; i++){

      std::cout << "DEPTH: " << currDepth << " and CHILD " << i << " has " << cubePointCounts[i] << " points. " << std::endl;

      if(!cubePointCounts[i]){ // no points in this cube
        std::cout << "SET CHILD : " << i << " TO NULL" << std::endl;
        child[i] = NULL;
        continue;
      } else {
        std::cout << "CHLID : " << i << " IS NOT NULL AND HAS POINTS:" << std::endl;
        for(size_t j=0; j<cubePointList[i].size(); j++){
          std::cout << "POINT: " << cubePointList[i][j] << std::endl;
        }
      }

      if(child[i] == NULL){
        child[i] = new Octree();
      } else {
        child[i]->clearOctree();
      }

      vec3 offset = boundsOffsetTable[i] * b.radius * 0.5;
      Bounds newBounds;
      newBounds.radius = b.radius * 0.5;
      newBounds.center = b.center + offset;

      std::cout << "offset: " << offset << std::endl;
      std::cout << "center is: " << newBounds.center << std::endl;
      std::cout << "box radius: " << newBounds.radius << std::endl;

      child[i]->radius = newBounds.radius;
      child[i]->center = newBounds.center;
      child[i]->height = currDepth + 1;
      child[i]->checker = checker;
      child[i]->qtype = qtype;

      child[i]->buildOctree(cubePointList[i], threshold, maxDepth, newBounds, currDepth + 1);

    }
    

    return true;
  }


  Bounds boundingBox(std::vector<vec3> points){

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
    vec3 radius = max - min; // length of the edge of square

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
}; 



class FCLDemo: public MzGlutApp {
public:

  ccd_real_t arena_radius;
  std::vector<vec3> points;
  std::vector<TransformedConvex*> objects;

  std::vector<vec3> pos_rate;
  std::vector<vec3> rot_rate;

  std::vector<Report> reports;
  std::vector<CollidingObjects> fclReport;

  std::vector< bool > colliding;
  std::vector< bool > fclCollision;
  std::vector<vec3> pointCloud;

  Bounds bound; // for boundingBoxTest
  Octree* octRoot;

  Checker checker;
  
  QueryType qtype;
  ccd_real_t dmin;

  DrawHelper helper;

  bool animating;
  bool draw_points;
  bool draw_spheres;
  bool wireframe;

  static vec3 randVec() {
    return vec3( mt_genrand_real1()*2-1,
                 mt_genrand_real1()*2-1,
                 mt_genrand_real1()*2-1 );
  }


  FCLDemo(int argc, char** argv):
    MzGlutApp(argc, argv) 
  {

    initWindowSize(640, 480);
    createWindow("FCL Demo");
    setupBasicLight(vec4f(1,1,1,0));

    camera.aim(vec3f(0, 0, 6),
               vec3f(0, 0, 0),
               vec3f(0, 1, 0));

    camera.setPerspective();

    camera.setHomePosition();

    cubePoints(20, points);

    arena_radius = 1.5;

    // make a box
    objects.push_back(transform(new Box(vec3(0.5)), Transform3(vec3(-1, 0, 0))));
    // objects.push_back(transform(new Box(vec3(0.5)), Transform3(vec3(0, -1, 1))));

    // make a point cloud
    vec3 point1 = vec3(-1, 0.1, 0.1);
    vec3 point2 = vec3(1, 0, 0);
    vec3 point3 = vec3(-0.5, 0, 1);
    vec3 point4 = vec3(0, 0, -1);
    vec3 point5 = vec3(-0.5, 1, -0.5);
    vec3 point6 = vec3(0, 1, 1);
    vec3 point7 = vec3(-0.5, 1, 0);

    // add to point cloud vector
    pointCloud.push_back(point1);
    pointCloud.push_back(point2);
    pointCloud.push_back(point3);
    pointCloud.push_back(point4);
    pointCloud.push_back(point5);
    pointCloud.push_back(point6);
    pointCloud.push_back(point7);


    for (size_t i=0; i<objects.size(); ++i) {
      pos_rate.push_back( 0.02 * randVec() );
      rot_rate.push_back( 0.02 * randVec() );
    }

    colliding.resize( objects.size(), false );
    fclCollision.resize(objects.size(), false);

    animating = false;
    draw_points = false;
    draw_spheres = false;
    wireframe = true;

    qtype = QUERY_PENETRATION;
    dmin = 3;

    octRoot = new Octree();
    octRoot->height = 0;
    octRoot->isLeaf = false;
    octRoot->checker = checker;
    octRoot->qtype = qtype;

    checkAll();
    fclCheck();
    
    setTimer(20, 0);

  }

  void checkAll() {

    colliding.clear();
    colliding.resize(objects.size(), false);
    reports.clear();

    Report report;

    size_t queries = 0;
    TimeStamp start = TimeStamp::now();

    for (size_t i=0; i<objects.size(); ++i) {
      Convex* c1 = objects[i];
      for (size_t j=0; j<i; ++j) {
        Convex* c2 = objects[j];
        bool collides = checker.query(qtype, &report, c1, c2, dmin);
        ++queries;
        if (collides) { 
          colliding[i] = colliding[j] = true;
        }
        if (report.flags & (HAVE_POSITION | HAVE_SEPARATION)) {
          reports.push_back(report);
        }
      }
    }

    TimeStamp end = TimeStamp::now();
    double elapsed = (end-start).toDouble();

    if (1) {
      std::cout << "did " << queries << " queries in " << elapsed << "s. "
                << "(" << (elapsed/queries) << " per query)\n";
    }

  }

  void boundingBoxTest(){
    // Find bounding box
    ccd_real_t xlo= 0; 
    ccd_real_t ylo = 0;
    ccd_real_t xhigh = 0;
    ccd_real_t yhigh = 0; 
    ccd_real_t zlo = 0;
    ccd_real_t zhigh = 0; 

    ccd_real_t x, y, z;

    for (size_t i=0; i<pointCloud.size(); ++i) {
      // std::cout << "Point " << i << ": ";
      std::cout << pointCloud[i] << std::endl;

      x = pointCloud[i][0];
      y = pointCloud[i][1];
      z = pointCloud[i][2];

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

    bound.center = min + radius * 0.5;

    bound.radius = radius[0]; // first set to x
    if (bound.radius < radius[1]) bound.radius = radius[1];
    if (bound.radius < radius[2]) bound.radius = radius[2];

    bound.radius = bound.radius * 1.05; // fudge factor

    // std::cout << "Bounding vars, max: " << max << " and min: " << min << std::endl;
    // std::cout << "center is: " << bound.center << std::endl;
    // std::cout << "box radius: " << bound.radius << std::endl;
    
  }

  void fclCheck(){

    // boundingBoxTest();

    // clear the tree
    octRoot->clearOctree();

    // build the octree
    int threshold = 1;
    int maxDepth = 4;
    int currDepth = 0;

    Bounds bound1 = octRoot->boundingBox(pointCloud);
    octRoot->center = bound1.center;
    octRoot->radius = bound1.radius;



    std::cout << "BUILD TREE START" << std::endl;

    octRoot->buildOctree(pointCloud, threshold, maxDepth, bound1, currDepth);
   
    std::cout << "BUILD TREE FINISH" << std::endl;

    for (size_t i=0; i<3; ++i) {
      Octree* child= octRoot->child[i];
      if(child != NULL){
        std::cout << "CHLID : " << i << " IS NOT NULL AND HAS POINTS:" << std::endl;
        for(size_t j=0; j<child->points.size(); j++){
          std::cout << "POINT: " << child->points[j] << std::endl;
        }
        
      } else {
        std::cout << "```child " << i << " is null..." << std::endl;
      }
    }

    // clear fclCollisionReport
    fclReport.clear();
    octRoot->fclReport.clear();
    bool collides;

    // loop through objects and check for collisions
    for(size_t i=0; i<objects.size(); i++){
      collides = false;
      std::cout << "Now checking fcl for object: " << i << std::endl << std::endl;
      collides = octRoot->checkForCollisions(objects[i], i, fclReport);
      fclCollision[i] = collides;
      
    }

    std::cout << "size of fcl report is now: " << fclReport.size() << std::endl;
    for(size_t i=0; i<fclReport.size(); i++){
      std::cout << "object index: " << fclReport[i].objectIndex << std::endl;
      std::cout << "point indices: " << std::endl;
      for(size_t j=0; j<fclReport[i].collidingPoints.size(); j++){
        std::cout << "point " << j << ": " << fclReport[i].collidingPoints[j] << std::endl;
      }
    }

  }

  virtual void timer(int value) {

    if (animating) {

      for (size_t i=0; i<objects.size(); ++i) {

        TransformedConvex* c = objects[i];

        quat q = c->xform.rotation();
        vec3 p = c->xform.translation();

        p += pos_rate[i];
        q = quat::fromOmega(rot_rate[i]) * q;
        q /= q.norm();

        for (int axis=0; axis<3; ++axis) {
          if ( (p[axis] > arena_radius && pos_rate[i][axis] > 0) ||
               (p[axis] < -arena_radius && pos_rate[i][axis] < 0) ) {
            pos_rate[i][axis] *= -1;
            rot_rate[i][axis] *= -1;
          }
        }

        c->xform = Transform3(q, p);

      }

      for (size_t i=0; i<pointCloud.size(); i++){
        // do something to get the points to move around???
      }

      checkAll();

      fclCheck();

    }

    glutPostRedisplay();
    setTimer(20, 0);

  }

  size_t findShape(const Convex* c) { 
    for (size_t i=0; i<objects.size(); ++i) {
      if (static_cast<const Convex*>(objects[i]) == c) {
        return i;
      }
    }
    return -1;
  }

  virtual void display() {

    MzGlutApp::display();

    glPushAttrib(GL_POLYGON_BIT);

    if (wireframe) {
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    //glDisable(GL_CULL_FACE);
    
    for (size_t i=0; i<objects.size(); ++i) {
      vec3 color = ccolors[i % ncolors];
      if (colliding[i] || fclCollision[i]) {
        for (int i=0; i<3; ++i) {
          if (!color[i]) { color[i] = 0.75; }
        }
        color *= 0.75;
      }
      glstuff::color(color);
      objects[i]->render(helper);
    }

    // draw point cloud
    vec3 fwd(camera.modelview().row(2).trunc());
    glPointSize(2.0);
    glBegin(GL_POINTS);
    for (size_t i=0; i<pointCloud.size(); i++){
      glVertex3d(pointCloud[i][0], pointCloud[i][1], pointCloud[i][2]);
    }
    glEnd();

    // draw bounding box

    // TransformedConvex* boundedBox = transform(new Box(vec3(bound.radius)), Transform3(bound.center));
    // vec3 color = ccolors[5];
    // glstuff::color(color);
    // boundedBox->render(helper);
    // delete boundedBox;

    // draw first 8 children of bounded box

    // for (size_t i=0; i<8; ++i) {
    //   Octree* child= octRoot->child[i];
    //   if(child != NULL){
    //     // std::cout << "```drawing child " << i << std::endl;
    //     // std::cout << "radius: " << child->radius << " and center: " << child->center << std::endl;

    //     // to draw child cubes: 
    //     TransformedConvex* box = transform(new Box(vec3(child->radius)), Transform3(child->center));
    //     vec3 color = ccolors[i % ncolors];
    //     glstuff::color(color);
    //     box->render(helper);
    //     delete box;
    //   } else {
    //     // std::cout << "```child " << i << " is null..." << std::endl;
    //   }
    // }

    // to show centers of child cubes:
    // glPointSize(2.0);
    // glBegin(GL_POINTS);
    // for (size_t i=0; i<3; ++i) {
    //   Octree* child= octRoot->child[i];
    //   if(child != NULL){
    //     
    //     vec3 color = ccolors[i % ncolors];
    //     glColor3fv( color.v );
    //     glVertex3d(child->center[0], child->center[1], child->center[2]);
    //   } else {
    //     std::cout << "```child " << i << " is null..." << std::endl;
    //   }
    // }
    // glEnd();  



    glPopAttrib();

    if (draw_spheres) {
      glColor3ub(191,191,191);
      glPushAttrib(GL_POLYGON_BIT);
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      for (size_t i=0; i<objects.size(); ++i) {
        vec3 c;
        objects[i]->center(c);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glTranslated(c[0], c[1], c[2]);
        gluSphere(helper.getQuadric(), objects[i]->maxDist()+1e-3,
                  helper.slices, helper.sstacks);
        glPopMatrix();
      }
      glPopAttrib();
    }

    for (size_t i=0; i<reports.size(); ++i) {
      const Report& ri = reports[i];
      if (ri.flags & HAVE_POSITION) {
        glColor3ub(255,255,0);
      } else {
        glColor3ub(255,0,255);
      }
      glstuff::draw_cylinder(helper.getQuadric(), 
                             ri.pos2, ri.pos1, 0.02f);
      glstuff::color(ccolors[ findShape( ri.c1 ) % ncolors ] );
      glstuff::draw_arrow(helper.getQuadric(),
                          ri.pos1, ri.pos1 - 0.3 * ri.direction, 0.04f);
      glstuff::color(ccolors[ findShape( ri.c2 ) % ncolors ] );
      glstuff::draw_arrow(helper.getQuadric(),
                          ri.pos2, ri.pos2 + 0.3 * ri.direction, 0.04f);
    }

    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);

    glColor3ub(0,0,0);
    glutWireCube(2*arena_radius);

    if (draw_points) {

      vec3 p;
      vec3 fwd(camera.modelview().row(2).trunc());

      glPointSize(2.0);
      glBegin(GL_POINTS);
      for (size_t i=0; i<objects.size(); ++i) {
        vec3 color = ccolors[ i % ncolors ] * 0.25;
        glstuff::color(color);
        const Convex* c = objects[i];
        Convex* d = NULL;
        if (dmin) {
          c = d = dilate(c, 0.5*dmin);
        }
        for (size_t j=0; j<points.size(); ++j) {
          if (vec3::dot(fwd, points[j]) > 0) {
            c->support(points[j], p);
            glstuff::vertex(p);
          }
        }
        delete d;
      }
      glEnd();

    }
    
    glPopAttrib();

    const char* algs[] = {
      "GJK", "MPR"
    };

    const char* qtypes[] = {
      "intersect",
      "separation",
      "penetration",
    };

    std::ostringstream ostr;
    ostr << "Algorithm: " << algs[checker.algorithm] << "\n";
    ostr << "Query type: " << qtypes[qtype] << "\n";
    ostr << "Min dist: " << dmin << "\n";

    drawString(10, 20, ostr.str());

    glutSwapBuffers();

  }

  virtual void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case '\r':
    case '\n':
      animating = !animating;
      break;
    case 'p':
      draw_points = !draw_points;
      glutPostRedisplay();
      break;
    case 's':
      draw_spheres = !draw_spheres;
      glutPostRedisplay();
      break;
    case 'w':
      wireframe = !wireframe;
      glutPostRedisplay();
      break;
    case 'a':
      checker.algorithm = AlgorithmType((checker.algorithm+1)%NUM_ALGORITHMS);
      checkAll();
      glutPostRedisplay();
      break;
    case '+':
    case '=':
      dmin = std::min(dmin+ccd_real_t(0.125), 2*arena_radius);
      checkAll();
      glutPostRedisplay();
      break;
    case '-':
      dmin = std::max(dmin-ccd_real_t(0.125), ccd_real_t(0));
      checkAll();
      glutPostRedisplay();
      break;
    case 'q':
      qtype = QueryType((qtype+1)%NUM_QUERY_TYPES);
      checkAll();
      glutPostRedisplay();
      break;
    case 27: // ESC
      // clean up octree
      // delete octRoot;

      exit(0);
      break;
    case ' ': // SPACE
      camera.recallHomePosition();
      viewChanged(false);
      break;
    }
  }


};

int main(int argc, char** argv) {

  FCLDemo demo(argc, argv);
  demo.run();
  
  return 0;
}
