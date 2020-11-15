//
// Created by jeh on 11/8/19.
//

#include "rsdController.h"


rsdController::rsdController(int argc,char* argv[]) {

    rosHandle = new SubsAndPubs(argc, argv);
    rtdControl = new RTDEControlInterface(ip);
}

SubsAndPubs* rsdController::getRosHandle() {
    return rosHandle;
}

void rsdController::newMoveJ(const std::vector< double > & _q, double _speed, double _acceleration)
{
    bool moveJCheck;

    for(int i = 0; i < 5; i++)
    {
             if(rosHandle->getState() == HOLDING || rosHandle->getState() == STOPPING || rosHandle->getState() == ABORTING)
            return;
        moveJCheck = rtdControl->moveJ(_q, _speed, _acceleration);
        //std::cout << moveJCheck << std::endl;
        if(moveJCheck == false)
        {
            rtdControl->stopRobot();
            rtdControl->reuploadScript();
            rtdControl->reconnect();
        }
        if(i == 4)
        {
            rosHandle->setState(ABORTING);
        }
        else
            break;
    }




}


void rsdController::runController() {


    while(true)
    {
        newMoveJ(cameraPos, velocity, acceleration);
        newMoveJ(discardPos, velocity, acceleration);

    }
    //dropOffOrders();
    //pushAndPickEmptyBoxes();

    //rtdControl->moveJ(spinFromStandard, velocity, acceleration);
    //std::cout << "standard pos" << std::endl; 
    //rtdControl->moveJ(pushPos1, velocity, acceleration);
    //rtdControl->moveJ(pushPos2, velocity, acceleration);
    //rtdControl->moveJ(pushPos3, velocity, acceleration);

    //while(ros::ok())
    //{
        

    //Print robot position    
    //std::cout << rtdReceiver->getActualQ().at(0) << ", " << rtdReceiver->getActualQ().at(1) << ", "  << rtdReceiver->getActualQ().at(2) << ", " << rtdReceiver->getActualQ().at(3) << ", "  << rtdReceiver->getActualQ().at(4) << ", " << rtdReceiver->getActualQ().at(5) << std::endl;
    //sleep(3);
  

    //rosHandle->rosSpinOnce();
    //}

}

void rsdController::resetRobot()
{
    rtdControl->stopRobot();
    //std::cout << "ROBOT STOPPED" << std::endl; 
    rtdControl->reuploadScript(); 
}

void rsdController::dropOffOrders()
{
    newMoveJ(spinFromStandard, velocity, acceleration);
    newMoveJ(topBoxesPrePos, velocity, acceleration);
    newMoveJ(topBoxesPos1, slow_vel, slow_acc);

    rosHandle->setGripper(true);

    //rtdControl->moveJ(topBoxesPos2, velocity, acceleration);
    //rtdControl->moveJ(topBoxesPos3, velocity, acceleration);
    //rtdControl->moveJ(topBoxesPos4, velocity, acceleration);


    newMoveJ({1.45115, -1.4577, 1.76301, -1.94236, -1.39561, 4.24172}, velocity, acceleration);
    newMoveJ({2.12276, -0.708336, 0.97682, -2.05624, -1.39446, 4.86032}, velocity, acceleration);
    newMoveJ({2.14013, -0.492211, 0.977719, -2.20196, -1.40806, 4.92335}, velocity, acceleration);

    /*
    std::vector<std::vector<double>> path1;
    std::vector<double> pose1 = {1.45115, -1.4577, 1.76301, -1.94236, -1.39561, 4.24172, velocity, acceleration, blend1};
    std::vector<double> pose2 = {2.12276, -0.708336, 0.97682, -2.05624, -1.39446, 4.86032, velocity, acceleration, blend2};
    std::vector<double> pose3 = {2.14013, -0.492211, 0.977719, -2.20196, -1.40806, 4.92335, velocity, acceleration, blend3};
    path1.push_back(pose1);
    path1.push_back(pose2);
    path1.push_back(pose3);
    rtdControl->moveJ(path1);
    */

    rosHandle->setGripper(false);


    newMoveJ(topBoxesPos5, velocity, acceleration);

    newMoveJ(botBoxesPos1, velocity, acceleration); //Perhaps create a path here
    newMoveJ(botBoxesPrePos1, velocity, acceleration);
    newMoveJ(botBoxesPos2, slow_vel, slow_acc);

     rosHandle->setGripper(true);



    newMoveJ({1.23754, -1.39022, 2.09956, -2.33302, -1.3642, 4.02381}, velocity, acceleration);
    newMoveJ({1.96039, -0.439567, 0.280793, -1.63821, -1.39662, 4.71916}, velocity, acceleration);
    newMoveJ({1.98781, -0.172545, 0.282939, -1.78347, -1.39928, 4.7807}, velocity, acceleration);

    /*
    std::vector<std::vector<double>> path2;
    std::vector<double> pose7 = {1.23754, -1.39022, 2.09956, -2.33302, -1.3642, 4.02381, velocity, acceleration, blend1};
    std::vector<double> pose8 = {1.96039, -0.439567, 0.280793, -1.63821, -1.39662, 4.71916, velocity, acceleration, blend2};
    std::vector<double> pose9 = {1.98781, -0.172545, 0.282939, -1.78347, -1.39928, 4.7807, velocity, acceleration, blend3};
    path2.push_back(pose7);
    path2.push_back(pose8);
    path2.push_back(pose9);
    rtdControl->moveJ(path2);
    */

    rosHandle->setGripper(false);

    rtdControl->moveJ(botBoxesPos6, velocity, acceleration);
}

   

void rsdController::pushAndPickEmptyBoxes()
{

   /* BOXES TOWARDS RIGHT

    1.   2.0355, -0.908906, 1.48775, -2.20194, -1.51078, 4.84308
    2.   2.04685, -0.746991, 1.45881, -2.25322, -1.53475, 4.843
    GRIP
    3.pickBoxesPos3
    4.pickBoxesPos4


    BOXES TOWARDS LEFT 
    
    1.    1.89454, -0.694183, 1.09883, -1.98752, -1.52101, 4.64946
    2.    1.89325, -0.574566, 1.115, -2.09447, -1.52078, 4.64987
    GRIP
    3.
    4. */


    newMoveJ({1.76641, -0.249708, 0.248898, -1.74229, -1.61402, 6.05646}, velocity, acceleration);
    newMoveJ({1.76051, -0.153164, 0.28107, -1.76311, -1.56628, 6.08706}, velocity, acceleration);
    newMoveJ({1.84128, -0.434111, 0.829026, -1.96683, -1.56905, 6.14592}, 1.3, 1);

    newMoveJ({1.8342, -0.680515, 0.828069, -1.89103, -1.60888, 6.14573}, velocity, acceleration);
    newMoveJ({1.89454, -0.694183, 1.09883, -1.98752, -1.52101, 4.64946}, velocity, acceleration);
    newMoveJ({1.89325, -0.574566, 1.115, -2.09447, -1.52078, 4.64987}, slow_vel, slow_acc);

    /*
    std::vector<std::vector<double>> path;
    std::vector<double> pose1 = {1.76641, -0.249708, 0.248898, -1.74229, -1.61402, 6.05646, velocity, acceleration, blend1}; //PUSH
    std::vector<double> pose2 = {1.76051, -0.153164, 0.28107, -1.76311, -1.56628, 6.08706, velocity, acceleration, blend2}; //PUSH
    std::vector<double> pose3 = {1.84128, -0.434111, 0.829026, -1.96683, -1.56905, 6.14592, velocity, acceleration, blend2}; //PUSH
    std::vector<double> pose4 = {1.8342, -0.680515, 0.828069, -1.89103, -1.60888, 6.14573, velocity, acceleration, blend2}; //PUSH
    std::vector<double> pose5 = {1.89454, -0.694183, 1.09883, -1.98752, -1.52101, 4.64946, velocity, acceleration, blend2};
    std::vector<double> pose6 = {1.89325, -0.574566, 1.115, -2.09447, -1.52078, 4.64987, velocity, acceleration, blend3};
    path.push_back(pose1);
    path.push_back(pose2);
    path.push_back(pose3);
    path.push_back(pose4);
    path.push_back(pose5);
    path.push_back(pose6);
    rtdControl->moveJ(path);
    */

    rosHandle->setGripper(true);


    newMoveJ({1.89454, -0.694183, 1.09883, -1.98752, -1.52101, 4.64946}, velocity, acceleration);
    newMoveJ({1.87837, -0.909326, 1.14014, -1.85578, -1.5215, 4.6011}, velocity, acceleration);
    newMoveJ({1.49158, -1.20385, 1.90555, -2.29526, -1.41039, 4.27618}, velocity, acceleration);

    /*
    std::vector<std::vector<double>> path3;
    std::vector<double> pose14 = {1.89454, -0.694183, 1.09883, -1.98752, -1.52101, 4.64946, velocity, acceleration, blend1};
    std::vector<double> pose15 = {1.87837, -0.909326, 1.14014, -1.85578, -1.5215, 4.6011, velocity, acceleration, blend2};
    std::vector<double> pose16 = {1.49158, -1.20385, 1.90555, -2.29526, -1.41039, 4.27618, velocity, acceleration, blend3};  
    path3.push_back(pose14);
    path3.push_back(pose15);
    path3.push_back(pose16);
    rtdControl->moveJ(path3);
    */

    rosHandle->setGripper(false);


    newMoveJ({1.49159, -1.28999, 1.91236, -2.295, -1.41035, 4.27621}, velocity, acceleration);
    newMoveJ({1.6232, -1.20988, 1.80225, -2.29541, -1.40696, 4.32509}, velocity, acceleration);
    newMoveJ({1.61767, -1.06374, 1.80391, -2.32337, -1.43873, 4.32485}, velocity, acceleration);
    newMoveJ({1.4055, -1.15581, 2.03164, -2.45051, -1.43752, 4.16297}, velocity, acceleration);

    /*
    std::vector<std::vector<double>> path1;
    std::vector<double> pose7 = {1.49159, -1.28999, 1.91236, -2.295, -1.41035, 4.27621, velocity, acceleration, blend1};
    std::vector<double> pose8 = {1.6232, -1.20988, 1.80225, -2.29541, -1.40696, 4.32509, velocity, acceleration, blend2};
    std::vector<double> pose9 = {1.61767, -1.06374, 1.80391, -2.32337, -1.43873, 4.32485, velocity, acceleration, blend2};
    std::vector<double> pose10 = {1.4055, -1.15581, 2.03164, -2.45051, -1.43752, 4.16297, velocity, acceleration, blend3};
    path1.push_back(pose7);
    path1.push_back(pose8);
    path1.push_back(pose9);
    path1.push_back(pose10);
    rtdControl->moveJ(path1);
    */

    newMoveJ({1.6232, -1.20988, 1.80225, -2.29541, -1.40696, 4.32509}, velocity, acceleration);
    newMoveJ({2.0355, -0.908906, 1.48775, -2.20194, -1.51078, 4.84308}, velocity, acceleration);
    newMoveJ({2.04459, -0.763472, 1.48784, -2.24696, -1.53542, 4.84294}, slow_vel, slow_acc);

    //old
    //2.04685, -0.746991, 1.45881, -2.25322, -1.53475, 4.843
    /*
    std::vector<std::vector<double>> path2;
    std::vector<double> pose11 = {1.6232, -1.20988, 1.80225, -2.29541, -1.40696, 4.32509, velocity, acceleration, blend1};
    std::vector<double> pose12 = {2.0355, -0.908906, 1.48775, -2.20194, -1.51078, 4.84308, velocity, acceleration, blend2};
    std::vector<double> pose13 = {2.04685, -0.746991, 1.45881, -2.25322, -1.53475, 4.843, velocity, acceleration, blend3};  
    path2.push_back(pose11);
    path2.push_back(pose12);
    path2.push_back(pose13);
    rtdControl->moveJ(path2);
    */

    rosHandle->setGripper(true);



    newMoveJ(pickBoxesPos3, velocity, acceleration);
    newMoveJ(pickBoxesPos4, velocity, acceleration);

    rosHandle->setGripper(false);


    newMoveJ({1.49159, -1.28999, 1.91236, -2.295, -1.41035, 4.27621}, velocity, acceleration);
    newMoveJ({1.6232, -1.20988, 1.80225, -2.29541, -1.40696, 4.32509}, velocity, acceleration);
    newMoveJ({1.61767, -1.06374, 1.80391, -2.32337, -1.43873, 4.32485}, velocity, acceleration);
    newMoveJ({1.59206, -1.07459, 1.84526, -2.35721, -1.46233, 4.37018}, velocity, acceleration);


    /*
    std::vector<std::vector<double>> path4;
    std::vector<double> pose17 = {1.49159, -1.28999, 1.91236, -2.295, -1.41035, 4.27621, velocity, acceleration, blend1};
    std::vector<double> pose18 = {1.6232, -1.20988, 1.80225, -2.29541, -1.40696, 4.32509, velocity, acceleration, blend2};
    std::vector<double> pose19 = {1.61767, -1.06374, 1.80391, -2.32337, -1.43873, 4.32485, velocity, acceleration, blend2};
    std::vector<double> pose20 = {1.59206, -1.07459, 1.84526, -2.35721, -1.46233, 4.37018, velocity, acceleration, blend3};
    path4.push_back(pose17);
    path4.push_back(pose18);
    path4.push_back(pose19);
    path4.push_back(pose20);
    rtdControl->moveJ(path4);
    */

    newMoveJ(pickBoxesPos19, velocity, acceleration);
    newMoveJ(pickBoxesPos20, velocity, acceleration);
    newMoveJ(standardPos, velocity, acceleration);
}


void rsdController::robotSleep(double _sleepTick)
{
    double tickCounter = 0;
    while(tickCounter < _sleepTick)
        tickCounter++;
}

bool rsdController::getIdleFlag() {
    return idleFlag;
}

void rsdController::commenceWork() {

    rosHandle->setGripper(false);
    char brickColor;
    //newMoveJ(standardPos, velocity, acceleration);
    int signal = rosHandle->getMESSignal();
    if(getIdleFlag() && rosHandle->waitFlag == false)
    {
        signal = M_IDLE;
        rosHandle->waitFlag = true;
    }
    if(rosHandle->waitFlag)
    {
        switch (signal)
        {
            case M_IDLE:
                rosHandle->waitFlag = false;
                rosHandle->setMESSignal(M_IDLE);
                sleep(2);
                break;
            case M_CLEAR:
                iterator = 0;
                rosHandle->waitFlag = false;
                setIdleFlag(false);
                rosHandle->setMESSignal(READY);
                break;
            case TAKEN:
                rosHandle->waitFlag = false;
                //Do robot routine, something something
                //Look up different colors for job, maybe a vector filled with different colors on a separate topic or look directly at database
                colorVector = rosHandle->getBrickVector();
                for(int i = 0; i < colorVector.size(); i++)
                {
                    std::cout << colorVector.at(i) << " ";
                }
                std::cout << std::endl;

                for(iterator; iterator < colorVector.size(); iterator++)
                {
                    if(rosHandle->getState() == STOPPING || rosHandle->getState() == ABORTING || rosHandle->getState() == RESETTING)
                        return;
                    brickColor = colorVector.at(iterator);
                    switch(brickColor)
                    {
                        case 'Y':
                            moveToFeederY();
                            //Do vision
                            moveToCamera(C_YELLOW);
                            if(rosHandle->getPISignal() == "True")
                                moveToTray(getJobCounter());
                            else
                            {
                                //TODO Maybe repick when discarding
                                moveToDiscard();
                                for(int i = 0; i < REPICKS+1; i++)
                                {
                                    if(rosHandle->getState() == STOPPING || rosHandle->getState() == ABORTING || rosHandle->getState() == RESETTING)
                                        return;
                                    std::cout << "No of repicks: " << i << std::endl;
                                    if(i < REPICKS)
                                    {
                                        moveToFeederY();
                                        //Do vision
                                        moveToCamera(C_YELLOW);
                                        if(rosHandle->getPISignal() == "True")
                                        {
                                            moveToTray(getJobCounter());
                                            break;
                                        }
                                        else
                                            moveToDiscard();
                                    }
                                    else
                                    {
                                        rosHandle->setState(HOLDING);
                                        rosHandle->setGUI(TOGGLE_HOLD);
                                        std::cout << "State rsd: " << rosHandle->getState();
                                        rosHandle->waitFlag = true;
                                        iterator = iterator - 1;
                                        return;
                                    }
                                }
                            }
                            break;

                        case 'R':
                            moveToFeederR();
                            //Do vision
                            moveToCamera(C_RED);
                            if(rosHandle->getPISignal() == "True")
                                moveToTray(getJobCounter());
                            else
                            {
                                moveToDiscard();
                                for(int i = 0; i < REPICKS+1; i++)
                                {
                                    if(rosHandle->getState() == STOPPING || rosHandle->getState() == ABORTING || rosHandle->getState() == RESETTING)
                                        return;
                                    std::cout << "No of repicks: " << i << std::endl;
                                    if(i < REPICKS)
                                    {
                                        moveToFeederR();
                                        //Do vision
                                        moveToCamera(C_RED);
                                        if(rosHandle->getPISignal() == "True")
                                        {
                                            moveToTray(getJobCounter());
                                            break;
                                        }
                                        else
                                            moveToDiscard();
                                    }
                                    else
                                    {
                                        rosHandle->setState(HOLDING);
                                        rosHandle->setGUI(TOGGLE_HOLD);
                                        std::cout << "State rsd: " << rosHandle->getState();
                                        rosHandle->waitFlag = true;
                                        iterator = iterator - 1;
                                        return;
                                    }
                                }
                            }
                            break;

                        case 'B':
                            std::cout << "Iterator: " << iterator << std::endl;
                            moveToFeederB();
                            //Do vision
                            moveToCamera(C_BLUE);
                            if(rosHandle->getPISignal() == "True")
                                moveToTray(getJobCounter());
                            else
                            {
                                moveToDiscard();
                                for(int i = 0; i < REPICKS+1; i++)
                                {
                                    if(rosHandle->getState() == STOPPING || rosHandle->getState() == ABORTING || rosHandle->getState() == RESETTING)
                                        return;
                                    std::cout << "No of repicks: " << i << std::endl;
                                    if(i < REPICKS)
                                    {
                                        moveToFeederB();
                                        //Do vision
                                        moveToCamera(C_BLUE);
                                        if(rosHandle->getPISignal() == "True")
                                        {
                                            moveToTray(getJobCounter());
                                            std::cout << "CORRECT BLUE" << std::endl;
                                            break;
                                        }
                                        else
                                            moveToDiscard();
                                    }
                                    else
                                    {
                                        rosHandle->setState(HOLDING);
                                        rosHandle->setGUI(TOGGLE_HOLD);
                                        std::cout << "State rsd: " << rosHandle->getState();
                                        rosHandle->waitFlag = true;
                                        iterator = iterator - 1;
                                        return;
                                    }
                                }
                            }
                            break;
                    }
                }

                    rosHandle->setTicket(rosHandle->getCurrentTicket());
                    rosHandle->setId(rosHandle->getCurrentId());
                    rosHandle->setMESSignal(COMPLETED);
                break;

            case DELETED:
                iterator = 0;
                rosHandle->waitFlag = false;
                incrementJobCounter();
                completedOrders++;
                sleep(2);
                setIdleFlag(true);
                break;
            case ERROR:
                rosHandle->waitFlag = false;
                sleep(4);
                setIdleFlag(true);
                break;

        }
    }
}

void rsdController::setIdleFlag(bool input) {
    idleFlag = input;
}

void rsdController::moveToFeederY()
{
    counter_Y = counter_Y + 1;
    newMoveJ(feederPrePosY, velocity, acceleration);
    newMoveJ(feederPrePos2Y, velocity, acceleration);
    newMoveJ(feederPosY, velocity, acceleration);
    rosHandle->setGripper(true);
    newMoveJ(feederPrePos2Y, velocity, acceleration);
    newMoveJ(feederPrePosY, velocity, acceleration);
    newMoveJ(standardPos, velocity, acceleration);
}

void rsdController::moveToFeederR()
{
    counter_R = counter_R + 1;
    newMoveJ(feederPrePosR, velocity, acceleration);
    newMoveJ(feederPrePos2R, velocity, acceleration);
    newMoveJ(feederPosR, velocity, acceleration);
    rosHandle->setGripper(true);
    newMoveJ(feederPrePos2R, velocity, acceleration);
    newMoveJ(feederPrePosR, velocity, acceleration);
    newMoveJ(standardPos, velocity, acceleration);
}

void rsdController::moveToFeederB()
{
    counter_B = counter_B + 1;
    newMoveJ(feederPrePosB, velocity, acceleration);
   // std::cout << "bub1" << std::endl; 
    newMoveJ(feederPrePos2B, velocity, acceleration);
    // std::cout << "bub2" << std::endl; 
    newMoveJ(feederPosB, velocity, acceleration);
    // std::cout << "bub3" << std::endl; 
    rosHandle->setGripper(true);
    newMoveJ(feederPrePos2B, velocity, acceleration);
    // std::cout << "bub4" << std::endl; 
    newMoveJ(feederPrePosB, velocity, acceleration);
    // std::cout << "bub5" << std::endl; 
    newMoveJ(standardPos, velocity, acceleration);
    // std::cout << "bub6" << std::endl; 
}

void rsdController::moveToDiscard() 
{
    newMoveJ(discardPrePos, velocity, acceleration);
    newMoveJ(discardPos, velocity, acceleration);
    rosHandle->setGripper(false);
    newMoveJ(standardPos, velocity, acceleration);
}

void rsdController::moveToCamera(int color)
{
    newMoveJ(cameraPos, velocity, acceleration);
    usleep(SECONDS * 0.5);
    rosHandle->setColor(color);
    usleep(SECONDS * 0.5);
    newMoveJ(standardPos, velocity, acceleration);
}


void rsdController::moveToTray(int boxNumber) {

    int box = boxNumber;

    switch(box){
        case 1:
            newMoveJ(boxPrePos, velocity, acceleration);
            newMoveJ(box1Pos, velocity, acceleration);
            rosHandle->setGripper(false);
            newMoveJ(standardPos, velocity, acceleration);
            break;
        case 2:
            newMoveJ(boxPrePos, velocity, acceleration);
            newMoveJ(box2Pos, velocity, acceleration);
            rosHandle->setGripper(false);
            newMoveJ(standardPos, velocity, acceleration);
            break;
        case 3:
            newMoveJ(boxPrePos, velocity, acceleration);
            newMoveJ(box3Pos, velocity, acceleration);
            rosHandle->setGripper(false);
            newMoveJ(standardPos, velocity, acceleration);
            break;
        case 4:
            newMoveJ(boxPrePos, velocity, acceleration);
            newMoveJ(box4Pos, velocity, acceleration);
            rosHandle->setGripper(false);
            newMoveJ(standardPos, velocity, acceleration);
            break;
    }
}

int rsdController::getJobCounter() {
    return jobCounter;
}

void rsdController::incrementJobCounter() {
    jobCounter++;
    if(jobCounter%5 == 0)
        jobCounter = 1;
}




rsdController::~rsdController() {

}


