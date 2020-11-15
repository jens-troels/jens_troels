//
// Created by jeppe on 4/21/19.
//

#ifndef PROJECT_RRT_H
#define PROJECT_RRT_H
#include "tree.h"

//#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <algorithm>

#include <rw/models.hpp>
#include <rw/kinematics.hpp>
#include <rw/proximity.hpp>
#include <rw/math.hpp>
#include <rw/invkin.hpp> //Jacobian Inverse Kinematics solver
#include "global_def.h"
#include "fstream"

using namespace rw;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::math;

using namespace rwlibs::proximitystrategies;

class Rrt {
public:
    Rrt();
    Rrt(Device::Ptr device, CollisionDetector::Ptr col_detector, State state, WorkCell::Ptr wc, Eigen::Matrix<double,6,6> C);
    Tree* task_constrained_rrt(Tree* init_tree, double step_size, int nodes);
    bool RGD_new_config(Node* _qS, Node* _qNear);
    bool TS_new_config(Node* _qS, Node* _qNear);
    bool FR_new_config(Node* _qS, Node* _qNear);
    void retract_config(Node* _qS, Eigen::Matrix<double,6,1> _dx);
    bool in_collision(Q q);
    Eigen::Matrix<double,6,1> computeTaskError( Q qs, Q qnear);
    Eigen::Matrix<double,6,1> task_coordinates(Q qs, Q qnear);
    Q random_config();

    vector<Q> CBiRRT(Q qstart, Q qgoal, Eigen::Matrix<double,6,6> C, int constrained_method = RGD);
    Node* constrained_extend(Tree* T, Node* qnear, Node* qtarget, double stepsize, int constrained_method = RGD);
    vector<Q> extract_path(Tree* Ta, Node* qa_reached, Tree* Tb, Node* qb_reached);

    bool local_planner(Q qstart, Q qend);

    State get_state();
    void set_state(State state);
    void grip_frame(Frame* gripper, Frame* object);
    void release_bottle();


    bool path_col_expanded_binary(Q q1, Q q2);
    vector<Q> removeRedundantNodes(vector<Q> currentPath);
    pair<bool, vector<Q>> parabolicBlend(Q q1, Q q2, Q q3, int tau);
    pair<bool, vector<Q>> linearInterp(Q q1, Q q2);
    vector<Q> interpolateRobotPath(vector<Q> robotPath);
    Transform3D<> vectorToTrans(vector<float> input);



    vector<Q> JIKS(Q q_initial, Vector3D<> p_desired, Rotation3D<> rot_desired);
    vector<Q> JIKS(Q q_initial, Transform3D<> TbaseTCP_desired);

    Transform3D<> get_TBaseObj(Transform3D<> TCamObj, Q camera_q);   
    Transform3D<> get_desired_TCP_pos(Transform3D<> TCamObj, Q camera_q);

    Transform3D<> get_TBaseObj_fake(Transform3D<> TCamObj, Q camera_q);
    Transform3D<> get_desired_TCP_pos_fake(Transform3D<> TCamObj, Q camera_q);

    void move_snemand(Transform3D<> TBaseSnemand);
    void grip_snemand(Q q_grab);
    void release_snemand(Q q_release);

    void log_data_path(vector<vector<Q>> path, string filepath);




private:
    CollisionDetector::Ptr _col_detector;
    Device::Ptr _device;
    State _state;
    WorkCell::Ptr _wc;

    Eigen::Matrix<double,6,6> _C;
    Eigen::Matrix<double,6,1> _disp; //Desired displacement of end effector in task frame coordinates.


};


#endif //PROJECT_RRT_H
