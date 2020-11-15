//
// Created by jeh on 9/29/19.
//

#include "rsdController.h"



int main(int argc,char* argv[]){
  bool mirFlag;
  int packML_state; 
  bool flaggiBoi;
  std::cout << "bubbi" << std::endl; 
  sleep(5);
    
    while(true)
    {   
        std::cout << "bubbi" << std::endl; 
        flaggiBoi = false; 
        mirFlag = true;
        packML_state = IDLE; 
        rsdController *rsd = new rsdController(argc, argv);
        SubsAndPubs *rsdHandle=rsd->getRosHandle();
        rsdHandle->setState(packML_state);
        rsdHandle->setGUI(8);
        std::cout << "State before While: " << packML_state << std::endl; 
        ros::Rate rate(200);
        rsd->resetRobot(); 
       
        //rsd->runController();
        
        while(ros::ok()) { 
            
            if(flaggiBoi)
                break; 

            packML_state = rsdHandle->getState();
            std::cout << "State:" << packML_state << std::endl; 
            switch (packML_state) {
                case STOPPED: {
                    std::cout << "Robot stopped, waiting for reset signal" << std::endl;
                    rsdHandle->waitGugi(0.2, TOGGLE_STOP);
                    rsdHandle->setState(RESETTING);
                    break;
                }
                case RESETTING: {
                    rsd->resetRobot(); 
                    std::cout << "FLAGGIBOI" << std::endl; 
                    sleep(2);
                    flaggiBoi = true; 
                    break;
                }
                case STARTING: {
                    
                    rsdHandle->setState(EXECUTE);

                    break;
                }
                case IDLE: {
                    //WAIT FOR PUSHBUTTON IN GUI waitGugi()
                    std::cout << "IDLING" << std::endl;
                    //rsd->runController();
                    rsdHandle->waitStart(0.1, 5); // Gugi start signal 5?
                    rsdHandle->setState(STARTING);
                    break;
                }
                case SUSPENDING: {
                    rsdHandle->setState(SUSPENDED);
                    break;
                }
                case SUSPENDED: {
                    rsdHandle->setMirSignal(1);
                    sleep(2);
                    rsdHandle->waitMir(0.2,1); // 1 = MiR Signal arrived
                    rsdHandle->setState(UNSUSPENDING);
                    break;
                }
                case UNSUSPENDING: {
                    rsdHandle->setState(EXECUTE);
                    break;
                }
                case EXECUTE: {
                    
                    //rsd->runController(); 
                    if(mirFlag)
                    {
                        rsd->commenceWork();
                        //CALLMIR
                        if(rsd->completedOrders >= 4)
                        {
                            rsd->completedOrders = 0; 
                            mirFlag = false;
                            rsdHandle->setState(SUSPENDING);
                        }  

                    }
                    else
                    {
                        //Drop offload/onload MiR
                        if(rsdHandle->getMirSig() == 1)
                        {
                            std::cout << "Ankommet MiR" << std::endl; 
                            rsd->dropOffOrders();
                            rsd->pushAndPickEmptyBoxes();
                            rsdHandle->setMirSignal(2);
                            rsdHandle->setFakeMir(0); 
                            mirFlag = true;
                        }  
                        if(rsdHandle->getMirSig() == 3)
                        {

                            std::cout << "MiR charging, unavailable" << std::endl; 

                            rsdHandle->setState(SUSPENDING);
                        }  
                       
                    }
                    break;
                }
                case STOPPING: {
                    std::cout << "STOPPING" << std::endl;
                    sleep(1);
                    rsdHandle->setState(STOPPED);
                    break;
                }

                case ABORTING: {
                    std::cout << "ABORTING" << std::endl;
                    sleep(1);
                    rsdHandle->setState(ABORTED);
                    break;
                }
                case ABORTED: {
                    std::cout << "Aborted" << std::endl;
                    
                    if (rsdHandle->getEmSig() == 2) 
                        {
                        rsdHandle->setGUI(TOGGLE_STOP);
                        rsdHandle->setState(STOPPING);
                        }
                    break;
                }
                case CLEARING: {
                    break;
                }
                case HOLDING: {
                    std::cout << "HEJ I HOLDING " << std::endl; 
                    rsdHandle->setState(HELD); //TODO CHECK IF ROBOT HAS FINISHED ITS CURRENT PATH
                    break;
                }
                case HELD: {
                    rsdHandle->waitGugi(0.2, TOGGLE_HOLD);
                    rsdHandle->setState(EXECUTE);
                    break;
                }
                case UNHOLDING: {
                    break;
                }
                case COMPLETING: {
                    break;
                }
                case COMPLETE: {
                    break;
                }
            }
            ros::spinOnce();
        }
        delete rsd;
        delete rsdHandle; 



    }
    return 0;
}