#include "CCDWrapper.h"
#include <cmath>
#include <ctime>
#include <mzcommon/glstuff.h>
#include <mzcommon/MzGlutApp.h>
#include <mzcommon/mersenne.h>
#include <mzcommon/octree.h>
#include <mzcommon/TimeUtil.h>
#include <sstream>
#include <src/fakerave.h>

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

class spccFetchModelTest: public MzGlutApp {
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

  fakerave::BodyConvexArray cgeoms;
  fakerave::BodyConvexArray env_cgeoms;

  GLUquadric* quadric;

  fakerave::Transform3Array xforms;

  const fakerave::KinBody& kbody;
  const fakerave::KinBody& env_kbody;

  fakerave::vec4 defaultColor;
  std::vector<fakerave::vec4> hcolors;


  bool animating;
  bool draw_points;
  bool draw_spheres;
  bool wireframe;
  bool draw_kbody;
  bool draw_skeleton;
  bool draw_cgeoms;
  bool draw_com;



  static vec3 randVec() {
    return vec3( mt_genrand_real1()*2-1,
                 mt_genrand_real1()*2-1,
                 mt_genrand_real1()*2-1 );
  }


  spccFetchModelTest(int argc, char** argv, fakerave::KinBody& k, const fakerave::KinBody& ek):
    MzGlutApp(argc, argv),
    kbody(k),
    env_kbody(ek),
    animating(false)
  {

    initWindowSize(640, 480);
    createWindow("spccFetchModelTest");
    setupBasicLight(vec4f(1,1,1,0));

    defaultColor = fakerave::vec4(fakerave::vec3(0.5), 1.0);

    k.compileDisplayLists();
    kbody.getCGeoms(cgeoms);

    env_kbody.getCGeoms(env_cgeoms);
    std::cerr << "env_cgeoms is initialized with size " <<env_cgeoms.size() << "\n";
    
    xforms.resize(kbody.bodies.size());
    hcolors.resize(kbody.bodies.size(), defaultColor);
    
    float zmin =  1e5;
    float zmax = -1e5;

    for (size_t i=0; i<xforms.size(); ++i) {
      float fz = xforms[i].translation().z();
      zmin = std::min(zmin, fz);
      zmax = std::max(zmax, fz);
    }

    float zmid = 0.5*(zmin + zmax);

    camera.aim(vec3f(3, 0, zmid), 
         vec3f(0, 0, zmid), 
         vec3f(0, 0, 1));

    camera.setPerspective();

    camera.setHomePosition();


    quadric = gluNewQuadric();

    draw_kbody = true;
    draw_skeleton = true;
    draw_cgeoms = true;


    // draw_points = false;
    // draw_spheres = false;
    // wireframe = true;

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

  void drawCGeoms( const fakerave::BodyConvexArray& cgeoms, 
                   const fakerave::Transform3Array & xforms ){

    enum { ncolors = 6 };

    static const fakerave::vec3 ccolors[ncolors] = {
      fakerave::vec3(1.0, 0.0, 0.0),
      fakerave::vec3(1.0, 1.0, 0.0),
      fakerave::vec3(0.0, 1.0, 0.0),
      fakerave::vec3(0.0, 1.0, 1.0),
      fakerave::vec3(0.0, 0.0, 1.0),
      fakerave::vec3(1.0, 0.0, 1.0),
    };

    glMatrixMode(GL_MODELVIEW);
    glPushAttrib(GL_POLYGON_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    for (size_t i=0; i<cgeoms.size(); ++i) {
      const fakerave::BodyConvex& bc = cgeoms[i];
      size_t body_index = bc.first;
      const ccdw::TransformedConvex* c = bc.second;
      assert( body_index < xforms.size() );
      glPushMatrix();
      glstuff::mult_transform( xforms[body_index] );
      glColor3dv( ccolors[ (body_index % ncolors) ].v );
      c->render(helper);
      glPopMatrix();
    }

    glPopAttrib();

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

    TimeStamp start = TimeStamp::now();


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

    TimeStamp buildTreeTimeEnd = TimeStamp::now();
    double buildTreeTime = (buildTreeTimeEnd-start).toDouble();

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

    TimeStamp end = TimeStamp::now();
    double elapsed = (end-start).toDouble();

    std::cout << "It took me " << buildTreeTime << " seconds to build the tree and " << elapsed << " seconds to run SPCC." << std::endl;
    std::cout << "Building the tree took: " << (buildTreeTime/elapsed)*100 << "% of the overall time" << std::endl;
    std::cout << "other ratio: " << elapsed/buildTreeTime << std::endl;
    std::cout << "size of spcc report is now: " << spccReport.size() << std::endl;
    
    // std::cout << "Collision report: " << std::endl;
    // for(size_t i=0; i<spccReport.size(); i++){
    //   std::cout << "object index: " << spccReport[i].objectIndex << std::endl;
    //   std::cout << "point indices: " << std::endl;
    //   for(size_t j=0; j<spccReport[i].collidingPoints.size(); j++){
    //     std::cout << "point " << j << ": " << spccReport[i].collidingPoints[j] << std::endl;
    //   }
    // }

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

      spccCheck();

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

    glMatrixMode(GL_MODELVIEW);

    fakerave::Transform3Array env_xforms(env_kbody.bodies.size());
    
    if (draw_kbody) {
      kbody.render(xforms, defaultColor, &hcolors);
      env_kbody.render(env_xforms, defaultColor, &hcolors);
    }

    if (draw_cgeoms) {
      drawCGeoms( cgeoms, xforms );
    }

    drawCGeoms( env_cgeoms, env_xforms);

    if (draw_skeleton) {
      kbody.renderSkeleton(xforms, quadric);
                           
    }


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
    // delete boundedBox->child;
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
    //     delete box->child;
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
    case 'c':
      draw_com = !draw_com;
      glutPostRedisplay();
      break;
    case 'g':
      draw_cgeoms = !draw_cgeoms;
      glutPostRedisplay();
      break;
    case 'b':
      draw_kbody = !draw_kbody;
      if (!draw_kbody && !draw_skeleton) {
        draw_skeleton = true;
      }
      glutPostRedisplay();
      break;
    case 'k':
      draw_skeleton = !draw_skeleton;
      glutPostRedisplay();
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

  std::string robot_path = "../fetch_description/robots/fetch.kinbody.xml";

  // set up base env.
  // once depth cloud is up, take the box out of here
  fakerave::KinBody kbody;
  kbody.load(robot_path);

  fakerave::KinBody env_kbody;
  env_kbody.name = "env";
  env_kbody.finish();

  // make a floor
  fakerave::Body b;
  b.name = "floor";
  fakerave::Geom g;
  g.type = "box";
  g.boxExtents = fakerave::vec3(5, 5, 0.1); // width, depth, height
  g.diffuseColor = fakerave::vec4(fakerave::vec3(0.5), 1);
  g.haveColor = 1;
  g.xform = fakerave::Transform3(fakerave::vec3(0, 0, -0.05)); // move down by half 

  env_kbody.bodies.push_back(b);


  // make a box to get around
  fakerave::Geom g2;
  g2.type = "box";
  g2.boxExtents = fakerave::vec3(.44, .43, .68); // width, depth, height
  g2.diffuseColor = fakerave::vec4(fakerave::vec3(0.5), 1);
  g2.haveColor = 1;
  g2.xform = fakerave::Transform3(fakerave::vec3(.7, 0.0, .34));

  b.geoms.push_back(g2);
  b.cgeoms.push_back(g2);


  fakerave::Geom g3;
  g3.type = "box";
  g3.boxExtents = fakerave::vec3(.01, .01, .01); // width, depth, height
  g3.diffuseColor = fakerave::vec4(fakerave::vec3(0.5), 1);
  g3.haveColor = 1;

  // load test point cloud



  // here's a fake cloud
  // float x, y, z;

  // int step = 350;
  // int max = 2;

  // for(int i = 0; i < step; i++){
  //   x = (max * 1.0 * i)/step;
  //   for(int j = 0; j < step; j++){
  //     y = (max * 1.0 * j)/step;
  //     z = 1 - (pow(x, 2) + pow(y, 2));
  //     if(z < 0.9*arena_radius && z > -0.9*arena_radius){
  //       pointCloud.push_back(vec3(x, y, z));
  //       pointCloud.push_back(vec3(-x, -y, z));
  //       pointCloud.push_back(vec3(x, y, -z));
  //       pointCloud.push_back(vec3(-x, -y, -z));

  //     }

  //   }
  // }

  // std::cout << "size of points is: " << pointsFromCloud.size() << std::endl;

  // add point cloud
  /*
  for(unsigned int i = 0; i < pointsFromCloud.size(); i+=1) {
    g3.xform = fakerave::Transform3(pointsFromCloud.at(i));
    b.geoms.push_back(g3);
    b.cgeoms.push_back(g3);
  }
  */




  spccFetchModelTest demo(argc, argv, kbody, env_kbody);
  demo.run();
  
  return 0;
}
