//
// Created by jeh on 11/8/19.
//

#ifndef TESTPKG_RSDCONTROLLER_H
#define TESTPKG_RSDCONTROLLER_H

#define SMALL_MOVE 2
#define BIG_MOVE 5


#include "SubsAndPubs.h"
#include <rtde_control_interface.h>
#include <rtde_io_interface.h>
#include <rtde_receive_interface.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include "global_def.h"


using namespace ur_rtde;


class rsdController {
public:
    rsdController(int argc,char* argv[]);
    SubsAndPubs * getRosHandle();
    void runController();
    void dropOffOrders();
    void pushAndPickEmptyBoxes();
    void moveToFeederY();
    void moveToFeederR();
    void moveToFeederB();
    void moveToDiscard();
    void moveToCamera(int color);
    void moveToTray(int boxNumber);
    bool getIdleFlag();             //Old function
    void setIdleFlag(bool input);   //Old function
    void commenceWork();
    int getJobCounter();            //Old function, maybe needed for logging?
    void incrementJobCounter();    //Old function
    void modbusSend();
    void robotSleep(double _sleepTick);
    void resetRobot(); 
    void newMoveJ(const std::vector< double > & _q, double _speed, double _acceleration); 

    ~rsdController();

    int counter_R = 0;
    int counter_B = 0;
    int counter_Y = 0;
    int iterator = 0; 
    int completedOrders = 0; 

private:
    SubsAndPubs *rosHandle;
    RTDEControlInterface *rtdControl;
    
    std::string ip = "192.168.0.201";
    bool idleFlag = true;
    int jobCounter = 1;
    double gain = 300;
    double smooth = 0.1;
    std::string colorVector; //Dummy vector for now (move to SubsAndPubs class)
    double sleepTick = 2000;
    int gripperOutput = 4;
    double velocity = 2;
    double slow_vel = 0.25;
    double acceleration = 1.2;
    double slow_acc = 0.1;

    double blend1 = 0;
    double blend2 = 0;
    double blend3 = 0;

 

    //Positions of objects
    
    //Boxes on tray
    std::vector<double> boxPrePos = {1.53388, -1.52134, 2.16108, -0.643607, 1.83069, 2.45838};

    std::vector<double> box1Pos = {1.44553, -1.20548, 2.25739, -0.97864, 1.90564, 2.41762};
    std::vector<double> box2Pos = {1.50829, -1.31934, 2.41513, -0.958214, 1.95082, 2.38952};
    std::vector<double> box3Pos = {1.78623, -1.17306, 2.25999, -1.13385, 2.18391, 2.38946};
    std::vector<double> box4Pos = {1.69244, -1.11708, 2.07841, -0.940892, 2.15916, 2.47846};

    //Home
    std::vector<double> homePos = {0,-M_PI/2,0,-M_PI/2,0,0};

    //Standard pos, mid cell
    std::vector<double> standardPos = {0.8941, -1.1751, 1.7406, -0.5682, 1.3128, 2.3137};


    //Discard box
    std::vector<double> discardPrePos = {1.28014, -1.10063, 1.49568, -0.56752, 1.31207, 2.31375};
    std::vector<double> discardPos = {1.27983, -0.897487, 1.57047, -0.567412, 1.31209, 2.31374};

    //Camera
    std::vector<double> cameraPos = {0.939467, -1.03114, 2.27187, -1.25223, 2.9141, 2.31406};

    //Feeder positions
    std::vector<double> feederPrePosY = {0.906385, -0.742707, 1.72588, -0.964723, 1.31988, 2.31343};
    std::vector<double> feederPrePos2Y = {0.932923, -0.593317, 1.40137, -0.805503, 1.31986, 2.31341};
    std::vector<double> feederPosY = {0.93997, -0.548284, 1.31586, -0.778023, 1.32006, 2.31339};

    
    std::vector<double> feederPrePosR = {0.770908, -0.710582, 1.63322, -0.878635, 1.15488, 2.3134};
    std::vector<double> feederPrePos2R = {0.802272, -0.611504, 1.41455, -0.760931, 1.18164, 2.28854};
    std::vector<double> feederPosR = {0.820315, -0.535381, 1.28142, -0.760738, 1.18276, 2.28882};
    


    std::vector<double> feederPrePosB = {0.645661, -0.664154, 1.55661, -0.933374, 1.05339, 2.32735};
    std::vector<double> feederPrePos2B = {0.681357, -0.570787, 1.3395, -0.771426, 1.05105, 2.32715};
    std::vector<double> feederPosB = {0.700111, -0.494945, 1.19667, -0.719673, 1.05144, 2.32718};



    //Finished order, drop off MiR
    std::vector<double> spinFromStandard = {1.43617, -1.4585, 1.62558, -1.79705, -1.40662, 4.19307};
    std::vector<double> topBoxesPrePos = {1.42005, -1.32902, 1.98757, -2.29887, -1.39446, 4.19354};

    std::vector<double> topBoxesPos1 = {1.46095, -1.15179, 2.02882, -2.5477, -1.38083, 4.26908};
    std::vector<double> topBoxesPos2 = {1.45115, -1.4577, 1.76301, -1.94236, -1.39561, 4.24172}; //Gripper activate
    std::vector<double> topBoxesPos3 = {2.12276, -0.708336, 0.97682, -2.05624, -1.39446, 4.86032};
    std::vector<double> topBoxesPos4 = {2.14013, -0.492211, 0.977719, -2.20196, -1.40806, 4.92335};
    std::vector<double> topBoxesPos5 = {2.12276, -0.708336, 0.97682, -2.05624, -1.39446, 4.86032}; //Gripper release

    std::vector<double> botBoxesPos1 = {1.24895, -1.53351, 1.93118, -1.96341, -1.34362, 4.07602};
    std::vector<double> botBoxesPrePos1 = {1.23821, -1.32979, 2.09896, -2.33321, -1.35563, 3.97206};
    std::vector<double> botBoxesPos2 = {1.23724, -1.18295, 2.12429, -2.46781, -1.33813, 4.02399}; //Gripper activate
    std::vector<double> botBoxesPos3 = {1.23754, -1.39022, 2.09956, -2.33302, -1.3642, 4.02381};
    std::vector<double> botBoxesPos4 = {1.96039, -0.439567, 0.280793, -1.63821, -1.39662, 4.71916};
    std::vector<double> botBoxesPos5 = {1.98781, -0.172545, 0.282939, -1.78347, -1.39928, 4.7807}; //Gripper release
    std::vector<double> botBoxesPos6 = {1.96039, -0.439567, 0.280793, -1.63821, -1.39662, 4.71916};


    //Push Routine
    std::vector<double> pushPos1 = {1.76641, -0.249708, 0.248898, -1.74229, -1.61402, 6.05646};
    std::vector<double> pushPos2 = {1.76051, -0.153164, 0.28107, -1.76311, -1.56628, 6.08706};
    std::vector<double> pushPos3 = {1.84128, -0.434111, 0.829026, -1.96683, -1.56905, 6.14592};
    std::vector<double> pushPos4 = {1.8342, -0.680515, 0.828069, -1.89103, -1.60888, 6.14573};


    //Pick boxes from MiR after push routine
    std::vector<double> pickBoxesPos1 = {2.0355, -0.908906, 1.48775, -2.20194, -1.51078, 4.84308};
    std::vector<double> pickBoxesPos2 = {2.04685, -0.746991, 1.45881, -2.25322, -1.53475, 4.843}; //Gripper activate
    std::vector<double> pickBoxesPos3 = {2.06558, -1.11126, 1.46129, -2.00895, -1.52005, 4.82962};
    std::vector<double> pickBoxesPos4 = {1.49158, -1.20385, 1.90555, -2.29526, -1.41039, 4.27618}; //Gripper release
    std::vector<double> pickBoxesPos5 = {1.49159, -1.28999, 1.91236, -2.295, -1.41035, 4.27621}; //Up
    std::vector<double> pickBoxesPos6 = {1.6232, -1.20988, 1.80225, -2.29541, -1.40696, 4.32509}; //Back
    std::vector<double> pickBoxesPos7 = {1.61767, -1.06374, 1.80391, -2.32337, -1.43873, 4.32485}; //Down
    std::vector<double> pickBoxesPos8 = {1.4055, -1.15581, 2.03164, -2.45051, -1.43752, 4.16297}; //Push
    std::vector<double> pickBoxesPos9 = {1.6232, -1.20988, 1.80225, -2.29541, -1.40696, 4.32509};
    std::vector<double> pickBoxesPos10 = {1.89454, -0.694183, 1.09883, -1.98752, -1.52101, 4.64946};
    std::vector<double> pickBoxesPos11 = {1.89325, -0.574566, 1.115, -2.09447, -1.52078, 4.64987}; //Gripper activate
    std::vector<double> pickBoxesPos12 = {1.89454, -0.694183, 1.09883, -1.98752, -1.52101, 4.64946};
    std::vector<double> pickBoxesPos13 = {1.87837, -0.909326, 1.14014, -1.85578, -1.5215, 4.6011};
    std::vector<double> pickBoxesPos14 = {1.49158, -1.20385, 1.90555, -2.29526, -1.41039, 4.27618}; //Gripper release
    std::vector<double> pickBoxesPos15 = {1.49159, -1.28999, 1.91236, -2.295, -1.41035, 4.27621}; //Up
    std::vector<double> pickBoxesPos16 = {1.6232, -1.20988, 1.80225, -2.29541, -1.40696, 4.32509};//Back
    std::vector<double> pickBoxesPos17 = {1.61767, -1.06374, 1.80391, -2.32337, -1.43873, 4.32485};//Down
    std::vector<double> pickBoxesPos18 = {1.59206, -1.07459, 1.84526, -2.35721, -1.46233, 4.37018};//Push
    std::vector<double> pickBoxesPos19 = {1.6232, -1.20988, 1.80225, -2.29541, -1.40696, 4.32509}; //Set to standard pos after this pos
    std::vector<double> pickBoxesPos20 = {1.60659, -1.40083, 1.78774, -2.14334, -1.46239, 4.36956}; 

    //Box discard after pickup
    //INDSÆT DISCARDPREPOS
    std::vector<double> discardBoxesPos1 = {1.28339, -0.874759, 1.61734, -1.44095, 1.36928, 2.29632};


    uint16_t tab_reg[32];


};


#endif //TESTPKG_RSDCONTROLLER_H