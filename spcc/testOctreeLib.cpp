#include "CCDWrapper.h"
#include <cmath>
#include <ctime>
#include <mzcommon/glstuff.h>
#include <mzcommon/MzGlutApp.h>
#include <mzcommon/mersenne.h>
#include <mzcommon/octree.h>
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

class SPCCDemo: public MzGlutApp {
public:

  ccd_real_t arena_radius;
  std::vector<vec3> points;
  std::vector<TransformedConvex*> objects;

  std::vector<vec3> pos_rate;
  std::vector<vec3> rot_rate;

  std::vector<Report> reports;
  std::vector<CollidingObjects> spccReport;

  std::vector< bool > colliding;
  std::vector< bool > spccCollision;
  std::vector<vec3> pointCloud;

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


  SPCCDemo(int argc, char** argv):
    MzGlutApp(argc, argv) 
  {

    initWindowSize(1024, 768);
    createWindow("SPCC Demo");
    setupBasicLight(vec4f(1,1,1,0));

    camera.aim(vec3f(0, 0, 6),
               vec3f(0, 0, 0),
               vec3f(0, 1, 0));

    camera.setPerspective();

    camera.setHomePosition();

    cubePoints(20, points);

    arena_radius = 4;

    // make a box
    objects.push_back(transform(new Box(vec3(0.5)), Transform3(vec3(0, 0, 0))));
    // objects.push_back(transform(new Box(vec3(0.5)), Transform3(vec3(0, -1, 1))));

    float x, y, z;

    int step = 350;
    int max = 2;

    for(int i = 0; i < step; i++){
      x = (max * 1.0 * i)/step;
      for(int j = 0; j < step; j++){
        y = (max * 1.0 * j)/step;
        z = 1 - (pow(x, 2) + pow(y, 2));
        if(z < 0.9*arena_radius && z > -0.9*arena_radius){
          pointCloud.push_back(vec3(x, y, z));
          pointCloud.push_back(vec3(-x, -y, z));
          pointCloud.push_back(vec3(x, y, -z));
          pointCloud.push_back(vec3(-x, -y, -z));

        }

      }
    }

    std::cout << "Number of points: " << pointCloud.size() << std::endl;


    for (size_t i=0; i<objects.size(); ++i) {
      pos_rate.push_back( 0.02 * randVec() );
      rot_rate.push_back( 0.02 * randVec() );
    }

    colliding.resize( objects.size(), false );
    spccCollision.resize(objects.size(), false);

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


    spccCheck();
    
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

  void spccCheck(){

    octRoot->clearOctree();

    // build the octree
    int threshold = 1;
    int maxDepth = 4;
    int currDepth = 0;

    Bounds bound1 = octRoot->boundingBox(pointCloud);
    octRoot->center = bound1.center;
    octRoot->radius = bound1.radius;

    // clear the tree

    octRoot->buildOctree(pointCloud, threshold, maxDepth, bound1, currDepth);

    for (size_t i=0; i<3; ++i) {
      Octree* child= octRoot->child[i];
      if(child != NULL){
        for(size_t j=0; j<child->points.size(); j++){
        }
        
      }
    }

    // clear spccCollisionReport
    spccReport.clear();
    octRoot->spccReport.clear();
    bool collides;

    // loop through objects and check for collisions
    for(size_t i=0; i<objects.size(); i++){
      collides = false;
      collides = octRoot->checkForCollisions(objects[i], i, spccReport);
      spccCollision[i] = collides;
      
    }

    std::cout << "size of spcc report is now: " << spccReport.size() << std::endl;
    for(size_t i=0; i<spccReport.size(); i++){
      std::cout << "object index: " << spccReport[i].objectIndex << std::endl;
      std::cout << "point indices: " << std::endl;
      for(size_t j=0; j<spccReport[i].collidingPoints.size(); j++){
        std::cout << "point " << j << ": " << spccReport[i].collidingPoints[j] << std::endl;
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

      // spccCheck();

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
      if (colliding[i] || spccCollision[i]) {
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
      delete octRoot;

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

  SPCCDemo demo(argc, argv);
  demo.run();
  
  return 0;
}
