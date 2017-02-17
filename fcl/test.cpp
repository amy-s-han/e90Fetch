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

typedef struct {
  vec3 center;
  ccd_real_t radius;
} Bounds;


class FCLDemo: public MzGlutApp {
public:

  ccd_real_t arena_radius;
  std::vector<vec3> points;
  std::vector<TransformedConvex*> objects;

  std::vector<vec3> pos_rate;
  std::vector<vec3> rot_rate;

  std::vector<Report> reports;

  std::vector< std::vector<vec3> > opoints;

  std::vector< bool > colliding;
  std::vector<vec3> pointCloud;

  // std::vector<ccd_real_t> boundingBoxParam;
  Bounds b;

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

    // make a point cloud
    vec3 point1 = vec3(0, 1, -1);
    vec3 point2 = vec3(1, 0, 0);
    vec3 point3 = vec3(-2, 0, 1);
    vec3 point4 = vec3(0, -1, 0);
    vec3 point5 = vec3(0, 1, -2);

    // add to point cloud vector
    pointCloud.push_back(point1);
    pointCloud.push_back(point2);
    pointCloud.push_back(point3);
    pointCloud.push_back(point4);
    pointCloud.push_back(point5);


    for (size_t i=0; i<objects.size(); ++i) {
      pos_rate.push_back( 0.02 * randVec() );
      rot_rate.push_back( 0.02 * randVec() );
    }

    colliding.resize( objects.size(), false );
    
    animating = false;
    draw_points = false;
    draw_spheres = false;
    wireframe = true;

    qtype = QUERY_PENETRATION;
    dmin = 3;

    checkAll();
    
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

  void boundingBox(){
    // Find bounding box
    int xlo= 0; 
    int ylo = 0;
    int xhigh = 0;
    int yhigh = 0; 
    int zlo = 0;
    int zhigh = 0; 

    std::cout << xlo << ylo << xhigh << yhigh << zlo << zhigh  << std::endl;

    for (size_t i=0; i<pointCloud.size(); ++i) {
      std::cout << "Point " << i << ": ";
      std::cout << pointCloud[i] << std::endl;

      if (pointCloud[i][0] < xlo){
        xlo = pointCloud[i][0];
      } else if (pointCloud[i][0] > xhigh){
        xhigh = pointCloud[i][0];
      }

      if (pointCloud[i][1] < ylo){
        ylo = pointCloud[i][1];
      } else if (pointCloud[i][1] > yhigh){
        yhigh = pointCloud[i][1];
      }

      if (pointCloud[i][2] < zlo){
        zlo = pointCloud[i][2];
      } else if (pointCloud[i][2] > zhigh){
        zhigh = pointCloud[i][2];
      }

    }

    std::cout << "Bounding vars: (" << xlo << ", " << xhigh << "), (" << ylo << ", " << yhigh << "), (" << zlo << ", " << zhigh << ")";

    ccd_real_t xcenter = (xhigh - xlo) / 2.0;
    ccd_real_t ycenter = (yhigh - ylo) / 2.0;
    ccd_real_t zcenter = (zhigh - zlo) / 2.0;

    std::cout << "center is: " << xcenter << ", " << ycenter << ", " << zcenter << std::endl;

    ccd_real_t boxRadius = std::max( std::max(abs(xcenter), abs(ycenter)) , abs(zcenter));

    std::cout << "box radius: " << boxRadius << std::endl;

    
    b.center = vec3(xcenter, ycenter, zcenter);
    b.radius = boxRadius;

    // boundingBoxParam.push_back(xcenter);
    // boundingBoxParam.push_back(ycenter);
    // boundingBoxParam.push_back(zcenter);
    // boundingBoxParam.push_back(ccd_real_t(boxRadius));

  }

  void fclCheck(){

    std::cout << "Does this work?" << std::endl;

    boundingBox();



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
      if (colliding[i]) {
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

    TransformedConvex* boundedBox = transform(new Box(vec3(b.radius)), Transform3(b.center));
    vec3 color = ccolors[5];
    glstuff::color(color);
    boundedBox->render(helper);



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
