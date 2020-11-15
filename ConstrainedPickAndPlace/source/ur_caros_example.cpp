#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>
#include <fstream>

#include "ros/package.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

#include "rw/rw.hpp"
#include "rw/invkin.hpp"

#include "caros/serial_device_si_proxy.h"

#include "URrobot.h"
#include "tree.h"
#include "Rrt.h"
#include "helpfuncs.h"

#include <rw/models.hpp>
#include <rw/kinematics.hpp>
#include <rw/proximity.hpp>
#include <rw/math.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/loaders.hpp>



using namespace std;

using namespace rw;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::math;

using namespace rwlibs::proximitystrategies;

/***Callbacks***/
bool flag_pose_updated = false;
Transform3D<> TCamObj;
void transform_cb(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    vector<float> pose_array = msg->data;
    cout << pose_array[0] << endl;
    cout << "Received TCamObj from pose_estimation: " << endl;
    TCamObj = grp7::vectorToTrans(pose_array);
    cout << TCamObj << endl;
    flag_pose_updated = true;
}



void nikolaj_main(int argc, char** argv)
{
    ros::init(argc, argv, "URRobot");

    URRobot robot;

    //Testing computeTaskError:
    //    Q qnear(6,5,4,6,-1,-5,0); //Inv kin: [0.074, -0.811, 0.637, -1.597, 0.406, -0.128]
    //    Q qs(6,2,4,6,-1.2,-5,0);  //Inv kin: [-0.188, 0.791, 0.602, 1.650, 0.595, -0.202]

    //Q qnear(6,0,-1.573,0,-1.573,0,0); //(The configuration the robot is loaded with in robworkstudio)

    int method = RGD;

    Q qnear(6, -2.165, 4.812 , 4.013, -0.977, 1.570, -3.738);
    Node qNear(qnear);
    Tree test(&qNear);

    auto packagePath = ros::package::getPath("ur_caros_example");
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(packagePath + "/WorkCell/Scene.wc.xml");

    Device::Ptr device = wc->findDevice("UR5");
    State state = wc->getDefaultState();

    CollisionDetector::Ptr col_detector = new CollisionDetector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    Eigen::Matrix<double,6,6> C; //Not important to set this, since this demo only uses CBiRRT
    Rrt rrt_obj(device, col_detector, state, wc, C);

    //rrt_obj.task_constrained_rrt(&test,0.05,1);

    /***Demo begins***/
//    Q qstart(6,0, -1.5708, 0, -1.5708, 0, 0);
//    //Q qgrab(6,-1.06832, -2.4617, -0.497454, -1.75324, 1.5708, 0);
//   // Q qgrab(6,-0.8594, 4.04899, -1.50578, -2.54321, -0.8594, 3.14159); //horizontal grab
//    //Q qgrab(6,1.81595, -0.121684, -0.634305, 5.46838, -1.5708, 0.245149); //vertical grab
//    Q qint1(6,1.81595, -0.372418, -0.281312, 5.36612, -1.5708, 0.245149); //lift vertical 7 cm
//    Q qint2(6,1.29369, -0.193697, -0.723264, 5.62935, -1.5708, -0.277106);//move 40cm in y
//    Q qgoal(6,1.29369, -0.00205949, -0.940157, 5.6546, -1.5708, -0.277106);//down 7cm
//    // Q qstart(6,-1.03385, -1.91541, -0.501259, -1.75877, 1.50378, 0);
//    //Q qgrab(6,-1.03385, -2.47933, -0.501259, -1.75877, 1.50378, 0);
//   // Q qgoal(6,0.855194, -0.657134, 1.4178, -3.90226, 5.42799, -0);
//   //  Q qgoal(6,-5.19358, -1.73133, 5.49957-6.28, -2.19744, 1.5708, 4.2312);


    Q qstart(6,0, -1.5708, 0, -1.5708, 0, 0);
    Q qgrab(6,-1.05495, -3.01991, 0.634305, -2.32679, 1.5708, 0.51585);
    Q qint1(6,1.5435, -2.10253, -0.915757, -1.6941, 1.5708, 3.1143);
    Q qint2(6,1.54011, -2.08528, -1.30537, -0.100164, 1.59963, 3.1311);
   // Q qint2(6,1.53509, -1.94592, -1.68192, 0.136973, 1.60434, 3.12938); //tilt bottle
    Q qint3(6,-1.05495, -3.01991, 0.634305, -2.32679, 1.5708, 0.51585); //move bottle back
    Q qgoal(6,0, -1.5708, 0, -1.5708, 0, 0);  //go to init position again

//    Q qint1(6,1.81595, -0.372418, -0.281312, 5.36612, -1.5708, 0.245149); //lift vertical 7 cm
//    Q qint2(6,1.29369, -0.193697, -0.723264, 5.62935, -1.5708, -0.277106);//move 40cm in y
//    //Q qgoal(6,1.29369, -0.00205949, -0.940157, 5.6546, -1.5708, -0.277106);//down 7cm
//    // Q qstart(6,-1.03385, -1.91541, -0.501259, -1.75877, 1.50378, 0);
//    //Q qgrab(6,-1.03385, -2.47933, -0.501259, -1.75877, 1.50378, 0);
//   // Q qgoal(6,0.855194, -0.657134, 1.4178, -3.90226, 5.42799, -0);
//    Q qgoal(6,-5.19358, -1.73133, 5.49957-6.28, -2.19744, 1.5708, 4.2312);


     Eigen::Matrix<double,6,6> C_start; //Initially no constraint on how to get to the bottle
     C_start <<     0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0;

     Eigen::Matrix<double,6,6> C_lift;
     C_lift <<      1, 0, 0, 0, 0, 0,   //x,y  yaw,pitch constrained
                    0, 1, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 0;

     Eigen::Matrix<double,6,6> C_x; //y,z yaw,pitch constained
     C_x <<       0, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0,
                  0, 0, 1, 0, 0, 0,
                  0, 0, 0, 1, 0, 0,
                  0, 0, 0, 0, 1, 0,
                  0, 0, 0, 0, 0, 0;

     Eigen::Matrix<double,6,6> C_y; //x,z yaw,pitch constrained
     C_y <<      1, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 1, 0, 0, 0,
                 0, 0, 0, 1, 0, 0,
                 0, 0, 0, 0, 1, 0,
                 0, 0, 0, 0, 0, 0;

     Eigen::Matrix<double,6,6> C_grab;
     C_grab <<       0, 0, 0, 0, 0, 0, //Constrain roll and pitch when moving bottle
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 1, 0, 0,
                     0, 0, 0, 0, 1, 0,
                     0, 0, 0, 0, 0, 0;

     Eigen::Matrix<double,6,6> C_tilt;
     C_tilt <<       1, 0, 0, 0, 0, 0, //
                     0, 1, 0, 0, 0, 0,
                     0, 0, 1, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 1, 0,
                     0, 0, 0, 0, 0, 1;

     vector<vector<Q>> full_path;

     vector<Q> temp_path = grp7::construct_path(qstart, qgrab, C_start, rrt_obj,method);
     full_path.push_back(temp_path);


     Frame *tcp_frame = wc->findFrame("UR5.TCP");
     Frame *bottle_frame = wc->findFrame("Bottle");
     device->setQ(qgrab,state);
     Kinematics::gripFrame(bottle_frame, tcp_frame,state);   //Grip the bottle
     rrt_obj.set_state(state);


     temp_path = grp7::construct_path(qgrab, qint1, C_grab, rrt_obj,method);
     full_path.push_back(temp_path);

     temp_path = grp7::construct_path(qint1, qint2, C_tilt, rrt_obj,method);//tilt
     full_path.push_back(temp_path);

     temp_path = grp7::construct_path(qint2, qint1, C_tilt, rrt_obj,method);//tilt back
     full_path.push_back(temp_path);

     temp_path = grp7::construct_path(qint1, qgrab, C_grab, rrt_obj,method);//back to bottle place
     full_path.push_back(temp_path);

     rrt_obj.release_bottle();

     temp_path = grp7::construct_path(qgrab, qstart, C_start, rrt_obj,method);//back to start
     full_path.push_back(temp_path);


     grp7::full_lua_parser_splicer(full_path,"lua_full_path.txt");


     //grp7::lua_parser_splicer(start_to_grab, grab_to_goal,"lua_combined_path.txt");

}

void jeppe_main()
{

}

void jens_main()
{

}

void soren_main(int argc, char** argv)
{
    ros::init(argc, argv, "URRobot");
    URRobot robot;

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("give_transform",10);
    ros::Subscriber sub2 = nh.subscribe("transform", 20, transform_cb);

    auto packagePath = ros::package::getPath("ur_caros_example");
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(packagePath + "/WorkCell/Scene.wc.xml");
    Device::Ptr device = wc->findDevice("UR5");
    State state = wc->getDefaultState();
    CollisionDetector::Ptr col_detector = new CollisionDetector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    Eigen::Matrix<double,6,6> C; //Not important to set this, since this demo only uses CBiRRT
    Rrt rrt_obj(device, col_detector, state, wc, C);

    Eigen::Matrix<double,6,6> C_start; //Initially no constraint on how to get to the bottle
    C_start <<     0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0;

    Eigen::Matrix<double,6,6> C_notilt;
    C_notilt <<     0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 1, 0, 0,
                   0, 0, 0, 0, 1, 0,
                   0, 0, 0, 0, 0, 0;

    Q q_upright    (6, 0, -1.573, 0, -1.573, 0, 0); //Q[6]{0, -1.573, 0, -1.573, 0, 0}
    Q q_take_image (6, 0.38, -1.546, 1.942, -1.326, -0.883, 0.842); //Q[6]{0.38, -1.546, 1.942, -1.326, -0.883, 0.842}
    Q q_take_img_upside_down (6, 0.229, -1.602, 1.96, -0.848, -0.748, 3.459 ); //Q[6]{0.229, -1.602, 1.96, -0.848, -0.748, 3.459}

    vector<Q> path_upright_to_take_image = rrt_obj.CBiRRT(q_upright, q_take_img_upside_down, C_start);



    //path_upright_to_take_image = rrt_obj.removeRedundantNodes(path_upright_to_take_image);

    //grp7::lua_parser(path_upright_to_take_image, "/home/soren/Desktop/lua_upright_to_take_image.txt");


    cout << "Trying to trigger pose_estimation - but first I go to sleep " << endl;
    //ros::Duration(10).sleep(); //Give the other nodes a chance to view changes: https://answers.ros.org/question/11167/how-do-i-publish-exactly-one-message/

    /***Demo on real robot. Go from home configuration to take image configuration***/
    //robot.setQ(q_upright);
    robot.setQ(q_take_img_upside_down); //Go to the grab image configuration

    /***Get the pose estimation***/
    std_msgs::Bool my_msg;
    my_msg.data = true;
    pub.publish(my_msg);
    cout << "Published the message" << endl;

    while(!flag_pose_updated) //Wait until pose_estimation finishes and sends back transform.
    {
        cout << "Waiting for the transform to be sent back" << endl;
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    //Transform3D<> TBaseTCP_des = rrt_obj.get_desired_TCP_pos(TCamObj, q_take_img_upside_down);
    Transform3D<> TBaseObj_fake = rrt_obj.get_TBaseObj_fake(TCamObj, q_take_img_upside_down);
    rrt_obj.move_snemand(TBaseObj_fake);

    Transform3D<> TBaseTCP_des = rrt_obj.get_desired_TCP_pos_fake(TCamObj, q_take_img_upside_down);

    vector<Q> q_desired = rrt_obj.JIKS(q_take_img_upside_down, TBaseTCP_des);
    Q q_grab;
    cout << "q_desired size: " << q_desired.size() << endl;
    if(q_desired.size())
    {
        q_grab = q_desired[0];
        cout << q_desired[0] << endl;
        //robot.setQ(q_desired[0]);
    }

    rrt_obj.grip_snemand(q_desired[0]);//Grip snemand
    Q q_goal1(6, 0.79, -1.675, 1.51, -1.406, -1.569, 5.304); //Q[6]{0.79, -1.675, 1.51, -1.406, -1.569, 5.304}
    Q q_goal2(6,1.438, -0.769, 0.259, -1.062, -1.569, 5.952); //Q[6]{1.438, -0.769, 0.259, -1.062, -1.569, 5.952}
    
    vector<Q> path0 = grp7::construct_path(q_take_img_upside_down, q_grab, C_start, rrt_obj); //Get from image position to grab position
    vector<Q> path1 = grp7::construct_path(q_grab, q_goal1, C_notilt, rrt_obj);
    vector<Q> path2 = grp7::construct_path(q_goal1, q_goal2, C_notilt, rrt_obj);
    vector<Q> path3 = grp7::construct_path(q_goal2, q_grab, C_notilt, rrt_obj); //Move back to qgrab (start position)
    rrt_obj.release_snemand(q_grab); //Release snemand
    vector<Q> path4 = grp7::construct_path(q_grab, q_take_img_upside_down, C_start, rrt_obj);



    vector<vector<Q>> complete_path = {path1, path2, path3};
    rrt_obj.log_data_path(complete_path, "/home/soren/Desktop/fullpath_data.txt");

    robot.follow_path(path0);
    robot.follow_path(path1);
    robot.follow_path(path2);
    robot.follow_path(path3);
    robot.follow_path(path4);

    grp7::lua_parser(path1, "/home/soren/Desktop/path1.txt");
    grp7::lua_parser(path2, "/home/soren/Desktop/path2.txt");
    grp7::lua_parser(path3, "/home/soren/Desktop/path3.txt");
    grp7::lua_parser(path4, "/home/soren/Desktop/path4.txt");


}

void vision_test(int argc, char** argv)
{
    ros::init(argc, argv, "URRobot");
    cout << "**********Vision test**********" << endl;


    auto packagePath = ros::package::getPath("ur_caros_example");
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(packagePath + "/WorkCell/Scene.wc.xml");
    Device::Ptr device = wc->findDevice("UR5");
    State state = wc->getDefaultState();
    CollisionDetector::Ptr col_detector = new CollisionDetector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    Eigen::Matrix<double,6,6> C; //Not important to set this, since this demo only uses CBiRRT
    Rrt rrt_obj(device, col_detector, state, wc, C);


    ofstream file_pose_estimates("pose_estimates_pos1.txt"); //[X Y Z R P Y] Pos1 = (0.075, -0.5,


    //URRobot robot;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("give_transform",10);
    ros::Subscriber sub2 = nh.subscribe("transform", 20, transform_cb);

    cout << "sleeping for 10 secs " << endl;
    ros::Duration(10).sleep();
    Q q_upright    (6, 0, -1.573, 0, -1.573, 0, 0);
    Q q_take_img_48_cm_away (6, 0.229, -1.602, 1.96, -0.848, -0.748, 3.459 ); //Q[6]{0.229, -1.602, 1.96, -0.848, -0.748, 3.459}
    Q q_take_img_80_cm_away (6,-0.079, -1.153, 1.035, -0.262, -1.026, 3.293); //Q[6]{-0.082, -1.156, 1.034, -0.264, -1.027, 3.292}

    Q q_img_0(6, 0.17, -1.429, 1.838, -0.415, -0.76, 3.091); //Q[6]{0.17, -1.429, 1.838, -0.415, -0.76, 3.091}

    //robot.setQ(q_upright);
    //robot.setQ(q_take_img_upside_down);

//    bool get_another_pose_estimate = true;
//    while(get_another_pose_estimate)

    int num_estimates = 20;
    for(int i = 0; i < num_estimates; i++)
    {
        std_msgs::Bool msg;
        msg.data = true;
        pub.publish(msg);           //Ask for a new transform:
        double time_elapsed = 0;
        while(!flag_pose_updated)   //Wait until pose_estimation finishes and sends back transform.
        {
            //cout << "Waiting to receive transform from pose_estimation: " << time_elapsed++ << "[s]" << endl;
            ros::Duration(1).sleep();
            ros::spinOnce();
        }
        flag_pose_updated = false;
        cout << "Iteration no.:" << i << endl << endl;
        Transform3D<> TBaseObj =  rrt_obj.get_TBaseObj(TCamObj, q_take_img_80_cm_away);
        grp7::log_data(TBaseObj, file_pose_estimates);
//        cout << "Press 1 to get another pose estimate. Press 0 to finish" << endl;
//        cin >> get_another_pose_estimate;
    }

    file_pose_estimates.close();


    cout << "**********Vision test termination**********" << endl;



}

int main(int argc, char** argv)
{


//   nikolaj_main(argc, argv);
//    jeppe_main();
//    jens_main();
    soren_main(argc, argv);
//    vision_test(argc, argv);


//    cout << "Hello world" << endl;

//    URRobot robot;
    return 0;
}

//Get grab base T TCP based on Matrix4f T camToObj






