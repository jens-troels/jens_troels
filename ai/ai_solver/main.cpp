#include <iostream>
#include "SokobanSolver.h"
#include <fstream>
#include <sstream>

int main() {

    ifstream myReadFile;
    myReadFile.open("../sokobanmap_hard.txt");
    string inputMap;
    if (myReadFile.is_open()) {
        while (!myReadFile.eof()) {

            inputMap+=myReadFile.get();

        }
    }
    cout<<inputMap<<endl;
    myReadFile.close();
    SokobanSolver solve(inputMap);
    solve.testFunc();


    return 0;

}