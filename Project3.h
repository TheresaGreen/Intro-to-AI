#ifndef Project3_hpp
#define Project3_hpp

#include <stdio.h>
#include <vector>
#include "Robot.h"
#include "Vector2D.h"
#include "Simulator.h"

class Project3 {
private:

public:
    /**
     * @brief default constructor
     */
    Project3(Simulator* sim1);

    /**
     * @brief get optimal action
     * @param sim1 simulator pointer
     * @param r robot pointer
     * @return optimal action
     */
    int H;
    int W;
    double pc;
    double otherp;
    double gamma;
    bool isvalid(int x, int y);
    void ValueIteration();
    int OptimalMove(int x, int y);
    bool FirstMove;
    double Update(int x, int y, int move);
    double RandF(int x, int y);
    
    
    double** reward;
    double** expectedU;
    double** holder;
    
    
    RobotAction getOptimalAction(Simulator* sim1, Robot* r1);
};

#endif
