//
// Created by jeh on 12/12/19.
//

#ifndef TESTPKG_GLOBAL_DEF_H
#define TESTPKG_GLOBAL_DEF_H

//For usleep
#define SECONDS 1000000

/////////////////
// MES DEFINES //
/////////////////

#define M_IDLE 0
#define M_CLEAR 1
#define READY 2
#define TAKEN 3
#define ERROR 4
#define COMPLETED 5
#define DELETED 6

//IO Lights
#define RED 0
#define YELLOW 1
#define GREEN 2

//Camera colors
#define C_BLUE 1
#define C_YELLOW 2
#define C_RED 3

//Error handling
#define REPICKS 3


///////////////////
// PackML States //
///////////////////
#define STOPPED 2
#define STARTING 3
#define IDLE 4
#define SUSPENDED 5
#define EXECUTE 6
#define STOPPING 7
#define ABORTING 8
#define ABORTED 9
#define HOLDING 10
#define HELD 11
#define RESETTING  100
#define SUSPENDING 101    //Not enumerated in tag guidelines
#define UNSUSPENDING 102
#define CLEARING 103
#define UNHOLDING 104
#define COMPLETING 105
#define COMPLETE 106

///////////////////
// Light States //
///////////////////
#define RED_SOLID 0
#define RED_FLASH 1
#define YELLOW_SOLID 6
#define YELLOW_FLASH 2
#define GREEN_SOLID 4
#define GREEN_FLASH 3
#define GREEN_YELLOW_FLASH 5
#define ALL_FLASH 7

//Toggle Button GUI
#define TOGGLE_HOLD 1
#define UNTOGGLE_HOLD 0
#define TOGGLE_STOP 2
#define UNTOGGLE_STOP 3
#define PUSH_RESET 4
#define PUSH_START 5


#endif //TESTPKG_GLOBAL_DEF_H
