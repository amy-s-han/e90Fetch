#include <sstream>
#include <cmath>
#include <typeinfo>
#include <fstream>
#include <string>
#include <iostream>
#include "stdint.h"
#include <Eigen/Core>

#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "FetchTrajectoryManager.h"
#include <mzcommon/MzGlutApp.h>
#include <mzcommon/mersenne.h>
#include <mzcommon/TimeUtil.h>
#include "mzcommon/octree.h"
#include "fetchImpl.h"
#include "common.h"

using namespace fakerave;


// Initialize lower limits for joint angles, upper limits for joint angles,
// start state for arm, end state for arm, and max step size
const Vector7 LOWER_DIMS = make_vec7(-1.61, -1.22, -3.14, -2.25, -3.14, -2.18, -3.14);
const Vector7 UPPER_DIMS = make_vec7(1.61, 1.52, 3.14, 2.25, 3.14, 2.18, 3.14);
const Vector7 START_STATE = make_vec7(1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0);
//const Vector7 GOAL_STATE = make_vec7(1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0001);
const Vector7 GOAL_STATE = make_vec7(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
const double MAX_STEP = 1*M_PI/180.0;

std::vector<ccdw::vec3> pointsFromCloud;

//const Vector7 START_STATE = make_vec7(1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0);
//const Vector7 START_STATE = make_vec7(.97, .9, 0.0, -.76, 0.0, 0.0, 0.0);
//const Vector7 GOAL_STATE = make_vec7(-.87, .9, -.59, -.76, 0.0, 0.0, 0.0);

enum {
  MAX_ITER = 10000,
  MAX_NODES = 100000,
  MAX_NODES_PER_EXTEND = 400, // 0=DISABLED
  DIFFS_PER_ADD = 30,
};

  
extern void fakerave::drawAxes(fakerave::real scale);

// Assume 200hz control rate for right now
const double fetch_dt = 1.0 / 200.0;


class RobotApp: public MzGlutApp {
public:

  const KinBody& kbody;
  const KinBody& env_kbody;

  const FetchTrajectory& trajectory;
  size_t cur_index;
  bool animating;

  Transform3Array xforms;

  vec4 defaultColor;

  ccdw::DrawHelper drawhelper;
  
  fakerave::BodyConvexArray cgeoms;
  fakerave::BodyConvexArray env_cgeoms;

  bool draw_kbody;
  bool draw_skeleton;
  bool draw_cgeoms;
  bool draw_com;

  size_t cur_manipulator;

  std::vector<vec4> hcolors;

  GLUquadric* quadric;

  RobotApp(int argc, char** argv, 
           KinBody& k, const KinBody& ek, const FetchTrajectory& traj):
    MzGlutApp(argc, argv),
    kbody(k),
    env_kbody(ek),
    trajectory(traj),
    cur_index(0),
    animating(false)
  {

    initWindowSize(640, 480);
    createWindow("Hello, Fetch!");
    setupBasicLight(vec4f(1,1,1,0));

    defaultColor = vec4(fakerave::vec3(0.5), 1.0);

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
    camera.setRotateType(GlCamera::ROTATE_2_AXIS);

    camera.setHomePosition();

    quadric = gluNewQuadric();

    draw_kbody = true;
    draw_skeleton = true;
    draw_cgeoms = true;
    std::cout << "here" << std::endl << std::flush;

    setTimer(40, 0); // 40 ms timer


  }

  void drawCGeoms( const fakerave::BodyConvexArray& cgeoms, 
                   const Transform3Array & xforms ){

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
      c->render(drawhelper);
      glPopMatrix();
    }

    glPopAttrib();

  }

  virtual void timer(int value) {
  	std::cout << "In timer" << std::endl << std::flush;

    if (animating) {
      glutPostRedisplay();
    } else {
      animating = false;
    }
    setTimer(40, 0);
  }

  virtual void display() {
  	std::cout << "In display" << std::endl << std::flush;

    MzGlutApp::display();

    glMatrixMode(GL_MODELVIEW);

    Transform3Array env_xforms(env_kbody.bodies.size());
    
    if (draw_kbody) {
      kbody.render(xforms, defaultColor, &hcolors);
      env_kbody.render(env_xforms, defaultColor, &hcolors);
    }

    if (draw_cgeoms) {
      drawCGeoms( cgeoms, xforms );
    }

    drawCGeoms( env_cgeoms, env_xforms);

    glClear(GL_DEPTH_BUFFER_BIT);

    if (draw_skeleton) {
      kbody.renderSkeleton(xforms, quadric);
                           
    }

    glutSwapBuffers();

  }           

 
  void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case '\n':
    case '\r':
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
    default:
      MzGlutApp::keyboard(key, x, y);
    }
  }


};

size_t fetch_duration_to_ticks(double duration) {
  return size_t(fabs(duration) / fetch_dt);
}

void ret_interp_joints(const Vector7& init, const Vector7& end, std::vector<Vector7>& toRet, size_t num_points){
  
  for(size_t i = 0; i < num_points; i++){
    double t = double(i+1)/num_points;
    double u = 3*t*t-2*t*t*t;
    Vector7 curr_vec;
    for(size_t j=0; j<7; j++){
     curr_vec[j] = (1-u)*init[j] + u*end[j];
    }
    toRet.push_back(curr_vec);
  }

}

void initialize(FetchTrajectoryManager& traj_man, std::vector<size_t>& joint_indices, size_t& num_joints, fakerave::Transform3& hand_desired, size_t& ind){
  
  // Initialize variables
  KinBody& kbody = traj_man.kbody;
  FetchTrajectoryElement& fte = traj_man.fte;

  
  // Name arm joints
  const char* names[] = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "upperarm_roll_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
    0,
  };

  num_joints = 7;
  joint_indices.assign(num_joints, 0.0);
  ind = kbody.lookupManipulator("arm");

  for (size_t i=0; i<num_joints; ++i) {
    // Make sure names were found
    // assert(kbody.lookupJoint(names[i])!=-1);
    joint_indices[i] = kbody.lookupJoint(names[i]);
  }

  // Get rid of sigularity
  //fte.robot_state.jvalues[joint_indices[1]] = 0.05;
  //fte.robot_state.jvalues[joint_indices[2]] = -0.10;

  // Update list of transformations
  kbody.transforms(fte.robot_state.jvalues, traj_man.xforms);

  // Get position that we would like the manipulatorIK to move joint values to
  fakerave::Transform3 hand_at = kbody.manipulatorFK(traj_man.xforms, ind);
  hand_desired = hand_at;

  // Move manipulator to initial position
  kbody.manipulatorIK(ind, hand_desired, fte.robot_state.jvalues, traj_man.xforms);
  kbody.transforms(fte.robot_state.jvalues, traj_man.xforms);

}

void validate_traj(FetchTrajectoryManager& traj_man, const Vector7& start, const Vector7& end){
  // Initialize variables
  KinBody& kbody = traj_man.kbody;
  FetchTrajectoryElement& fte = traj_man.fte;

  std::vector<size_t> joint_indices(7);
  size_t num_joints, ind;
  fakerave::Transform3 hand_desired;

  initialize(traj_man, joint_indices, num_joints, hand_desired, ind);

  std::vector<Vector7> toRet;
  ret_interp_joints(start, end, toRet, 100);
  std::cout << toRet.size() << std::endl;
  for (size_t i=0; i<100; i++){
    Vector7 curr_joints = toRet[i];
    for(size_t j=0; j<7; j++){
      kbody.transforms(fte.robot_state.jvalues, traj_man.xforms);
      fte.robot_state.jvalues[joint_indices[j]] = curr_joints[j];
      assert(fte.collisions.size() == 0);
      traj_man.collisionCheckAndAppend();
    }
  }

}

void move_to_zero(FetchTrajectoryManager& traj_man){
  // Initialize variables
  KinBody& kbody = traj_man.kbody;
  FetchTrajectoryElement& fte = traj_man.fte;
 
  std::vector<size_t> joint_indices(7);
  size_t num_joints, ind;
  fakerave::Transform3 hand_desired;
  
  initialize(traj_man, joint_indices, num_joints, hand_desired, ind);

  Vector7 joint_start = make_vec7(1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0);
  Vector7 prev = make_vec7(1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0);
  std::vector<size_t> joint_ind(7);
  std::vector<float> joint_pos(7);

  joint_start[0] = 1.32;
  joint_start[1] = 1.40;
  joint_start[2] = -0.2;
  joint_start[3] =  1.72;
  joint_start[4] =  0.0;
  joint_start[5] = 1.66;
  joint_start[6] = 0.0;
 
  prev = joint_start;
  for (size_t i=0; i<num_joints; i++) {
    kbody.transforms(fte.robot_state.jvalues, traj_man.xforms);
    fte.robot_state.jvalues[joint_indices[i]] = joint_start[i];
    traj_man.collisionCheckAndAppend();
  }

  joint_ind[0] = 2;
  joint_ind[1] = 0;
  joint_ind[2] = 2;
  joint_ind[3] = 5;
  joint_ind[4] = 1;
  joint_ind[5] = 3;
  joint_ind[6] = 2;

  joint_pos[0] = -1.99;
  joint_pos[1] = 0.0;
  joint_pos[2] = -3.14;
  joint_pos[3] = 0.0;
  joint_pos[4] = 0.0;
  joint_pos[5] = 0.0;
  joint_pos[6] = 0.0;

  std::ofstream myfile;
  myfile.open("../../../catkin_ws/src/fetch_cappy/e90/textfiles/to_zero.txt");
  
  size_t num_pos = 7;
  for (size_t i=0; i<num_pos; i++){
    joint_start[joint_ind[i]] = joint_pos[i];
    std::vector<Vector7> goTo;
    ret_interp_joints(prev, joint_start, goTo, 50);
    prev = joint_start;
    for (size_t k=0; k<50; k++){
      Vector7 joint_vals = goTo[k];
      for (size_t j=0; j<7; j++){
        kbody.transforms(fte.robot_state.jvalues, traj_man.xforms);
        fte.robot_state.jvalues[joint_indices[j]] = joint_vals[j];
        // Send values to output file      
        std::string output;
        double jvalue = fte.robot_state.jvalues[joint_indices[j]];
        std::ostringstream convert;
        convert << jvalue;
        output = convert.str();
        // If get to last joint go to next line on text file
        if ((j)==6){
       	  myfile << output+"\n";
        }
        else{
	  myfile << output+" ";
        }
      }
      traj_man.collisionCheckAndAppend();
    }
  }
  
  myfile.close();
  std::cout << "traj_man.traj.size() = " << traj_man.traj.size() << "\n";

}






void test_fetch_impl(FetchTrajectoryManager& traj_man, Octree* octRoot){
  // transpose prevents newlines here
  std::cout << "LOWER_DIMS is: " << LOWER_DIMS.transpose() << "\n";
  std::cout << "UPPER_DIMS is: " << UPPER_DIMS.transpose() << "\n";

  Vector7 range = UPPER_DIMS - LOWER_DIMS;

  std::cout << "UP - LO = " << (range).transpose() << "\n";
  std::cout << "||range|| (L2 norm) = " << range.norm() << "\n";
  std::cout << "||range|| (inf norm) = " << range.lpNorm<Eigen::Infinity>() << "\n";

  // modifies in-place
  vec7_normalize_lp(range);
  std::cout << "range relative to max: " << range.transpose() << "\n";

  // Initialize variables
  KinBody& kbody = traj_man.kbody;
  KinBody& env_kbody = traj_man.env_kbody;

  FetchImpl impl(true, kbody, env_kbody);

  impl.setOctrootChecker();
  impl.octRoot = octRoot;

  TimeStamp t = TimeStamp::now();
  mt_init_genrand(t.nsec);

}  


/**/
int main(int argc, char** argv) {

  std::string package_path = ros::package::getPath("fetchros") ;
  std::cout << package_path << std::endl;
  std::string robot_path = package_path + "/fetch_description/robots/fetch.kinbody.xml";


  // set up base env.
  // once depth cloud is up, take the box out of here
  KinBody kbody;
  kbody.load(robot_path);

  KinBody env_kbody;
  env_kbody.name = "env";
  env_kbody.finish();

  // make a floor
  Body b;
  b.name = "floor";
  Geom g;
  g.type = "box";
  g.boxExtents = fakerave::vec3(5, 5, 0.1); // width, depth, height
  g.diffuseColor = vec4(fakerave::vec3(0.5), 1);
  g.haveColor = 1;
  g.xform = fakerave::Transform3(fakerave::vec3(0, 0, -0.05)); // move down by half 

  // make a box to get around
  Geom g2;
  g2.type = "box";
  g2.boxExtents = fakerave::vec3(.44, .43, .68); // width, depth, height
  g2.diffuseColor = vec4(fakerave::vec3(0.5), 1);
  g2.haveColor = 1;
  g2.xform = fakerave::Transform3(fakerave::vec3(.7, 0.0, .34));

  Geom g3;
  g3.type = "box";
  g3.boxExtents = fakerave::vec3(.01, .01, .01); // width, depth, height
  g3.diffuseColor = vec4(fakerave::vec3(0.5), 1);
  g3.haveColor = 1;

 
  //b.geoms.push_back(g);
  //b.cgeoms.push_back(g);
  //b.geoms.push_back(g2);
  //b.cgeoms.push_back(g2);
  //b.geoms.push_back(g3);
  //b.cgeoms.push_back(g3);

  // read points in


  std::cout << "size of points is: " << pointsFromCloud.size() << std::endl;

  /*
  for(unsigned int i = 0; i < pointsFromCloud.size(); i+=1) {
    g3.xform = fakerave::Transform3(pointsFromCloud.at(i));
    b.geoms.push_back(g3);
    b.cgeoms.push_back(g3);
  }
  */

  env_kbody.bodies.push_back(b);
  //return -1;

  FetchTrajectoryManager traj_man(kbody, env_kbody);
  
  // build octree here
  Octree* octRoot = new Octree();
  octRoot->height = 0;
  octRoot->isLeaf = false;
  octRoot->qtype=ccdw::QUERY_PENETRATION;


  test_fetch_impl(traj_man, octRoot);

  //make_sinusoid(traj_man);
  //move_in_square(traj_man);
  //move_to_high_five(traj_man); 
  //move_to_zero(traj_man);
  //move_to_tuck(traj_man);

  //Vector7 start=make_vec7(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  //Vector7 end=make_vec7(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  //validate_traj(traj_man, start, end);

  RobotApp app(argc, argv, kbody, env_kbody, traj_man.traj);
  app.run();
  
  return 0;

}
