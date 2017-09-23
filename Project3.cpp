#include "Project3.h"
#include <iostream>


Project3::Project3(Simulator* sim1) {
    pc = 0.7;
    otherp = (1.00 - pc)/3;
    gamma = .9;
    
    double OBST = -0.01;
    double GOAL = 10.0;
    double REWARD = 0.0;
    
    
    FirstMove = true;
    //make and fill the value array with zeros everywhere
    H = sim1->getHeight();
    W = sim1->getWidth();
  
    
    //fill reward, holder, and expectedU
    reward = new double*[H];
    for (int i = 0; i <H; i++){
        reward[i] = new double[W];
    
    }
    for(int i=0; i < H; i++){
        for(int j = 0; j < W; j++){
        
            reward[i][j]=REWARD;
        }
    }
    
    holder = new double*[H];
    for (int i = 0; i <H; i++){
        holder[i] = new double[W];
        
    }
    for(int i=0; i < H; i++){
        for(int j = 0; j < W; j++){
            
            holder[i][j]=REWARD;
        }
    }
    
    expectedU = new double*[H];
    for (int i = 0; i <H; i++){
        expectedU[i] = new double[W];
        
    }
    for(int i=0; i < H; i++){
        for(int j = 0; j < W; j++){
            expectedU[i][j]=REWARD;
        }
    }
    
    
    std::vector<Point2D> obstacles = sim1-> getKnownObstacleLocations();
    
    for (int i =0; i <obstacles.size(); i++){
        int xVal = obstacles[i].x;
        int yVal = obstacles[i].y;
        reward[xVal][yVal]=OBST;
        holder[xVal][yVal]=OBST;
        expectedU[xVal][yVal]=OBST;
    
    }
    
    Point2D target = sim1->getTarget();
    int goalX = target.x;
    int goalY = target.y;
    
    reward[goalX][goalY]= GOAL;
    holder[goalX][goalY]= GOAL;
    expectedU[goalX][goalY]= GOAL;
    
    
    //TEST FOR WHAT THE MATRIX LOOKS LIKE
   /* for(int i=0; i < H; i++){
        for(int j = 0; j < W; j++){
            
            std::cout<<expectedU[i][j];
            
        }
        
        std::cout<<std::endl;
    }
    
   std::cout<<std::endl;
     std::cout<<std::endl;
     std::cout<<"END INIT"<<std::endl;
     std::cout<<std::endl;*/
    
    
}

/**
 * @brief get optimal action
 * @param sim simulator pointer
 * @param r robot pointer
 * @return optimal action
 */

bool Project3::isvalid(int x, int y){
    if(x<0 || x >= H ){
        return false;
    }
    if(y<0 || y >= W ){
        return false;
    }
    if(reward[x][y]<0){
        return false;
    
    }
    
    
    
    return true;

}



//Optimal move will go through your EXPECTED UTILITY matrix and determine the best move
//UP = 0, Right = 1, Down = 2; Left = 3;
int Project3::OptimalMove(int x, int y){

    //UP
    double UP= pc*RandF((x-1),y)+
                otherp*RandF(x, (y+1)) +
                otherp*RandF(x+1, y) +
                otherp*RandF(x, (y-1));
    //RIGHT
    double RIGHT = pc*RandF(x,(y+1))+
                    otherp*RandF((x+1), (y)) +
                    otherp*RandF(x, (y-1)) +
                    otherp*RandF((x-1), (y));
    //DOWN
    double DOWN = pc*RandF((x+1),(y))+
                    otherp*RandF((x), (y-1)) +
                    otherp*RandF((x-1), (y)) +
                    otherp*RandF((x), (y+1));
    //LEFT
    double LEFT=pc*RandF((x),(y-1))+
                otherp*RandF((x-1), (y)) +
                otherp*RandF((x), (y+1)) +
                otherp*RandF((x+1), (y));
    
    if (UP >= RIGHT && UP >= DOWN && UP >= LEFT){
        return 0;
    } if (RIGHT >= UP && RIGHT >= DOWN && RIGHT >= LEFT){
        return 1;
    } if (DOWN >= RIGHT && DOWN >= UP && DOWN >= LEFT){
        return 2;
    }
    //WHAT'S LEFT
    return 3;

}


double Project3::RandF(int x, int y){
   
    
    if(isvalid(x,y)){
        double end = reward[x][y];
        end = end + gamma * expectedU[x][y];
        return end;
        
    }
    
    //you hit a wall so there is no value for this
    return 0.0;

}

double Project3::Update(int x, int y, int move){
    double update = 0.0;
    
    //MOVE UP
    if(move ==0){
        update = pc*RandF((x-1),y)+
                 otherp*RandF(x, (y+1)) +
                 otherp*RandF(x+1, y) +
                 otherp*RandF(x, (y-1));
    
    }
    //MOVE RIGHT
    if(move ==1){
        update = pc*RandF(x,(y+1))+
                otherp*RandF((x+1), (y)) +
                otherp*RandF(x, (y-1)) +
                otherp*RandF((x-1), (y));
        
    }
    //MOVE DOWN
    if(move ==2){
        update = pc*RandF((x+1),(y))+
                otherp*RandF((x), (y-1)) +
                otherp*RandF((x-1), (y)) +
                otherp*RandF((x), (y+1));
                                                   
    }
    //MOVE LEFT
    if(move ==3){
    update = pc*RandF((x),(y-1))+
             otherp*RandF((x-1), (y)) +
             otherp*RandF((x), (y+1)) +
             otherp*RandF((x+1), (y));
                                                   
    }
    
    return update;


}




//////////////////////////////////////////////////////////////////////////////

void Project3::ValueIteration(){
    int Iterate = 100;
    
    //change number of times to iterate to make sure its a good enough estimation
    for (int i=0; i <Iterate; i++){
        //iterate through every square on the map
        for(int i=0; i < H; i++){
            for(int j = 0; j < W; j++){
                //determine what's the BEST move to make at that square
                //UP = 0, Right = 1, Down = 2; Left = 3;
                int move = OptimalMove(i,j);
                //calculate your NEW UTILITY and store it in HOLDER
                holder[i][j] = Update(i, j, move);
            }//end for W
            
        }//end for H
      
        for(int i=0; i < H; i++){
            for(int j = 0; j < W; j++){
                
                expectedU[i][j]=holder[i][j];
                
                //std::cout<<expectedU[i][j];
            }
            //std::cout<<std::endl;
            
        }

        //switch your expected Utility to your new updated Utility!
    } //end interation
   
       for(int i=0; i < H; i++){
            for(int j = 0; j < W; j++){
             
                expectedU[i][j]=holder[i][j];
                
                std::cout<<expectedU[i][j]<<" ";
            }
            std::cout<<std::endl;
    
        }
    
    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout<<"END: "<<std::endl;
    std::cout<<std::endl;


}


/////////////////////////////////////////////////////////////////////////////

RobotAction Project3::getOptimalAction(Simulator* sim1, Robot* r1) {
	// Here, you should find the next step of the robot.
	// The robot should always follow a shortest path (wrt the known and sensed obstacles) to the goal.
   
    
    if (FirstMove==true){
        ValueIteration();
        
        
        FirstMove=false;
    
    }
    
    Point2D current = r1->getPosition();
    int currentX = current.x;
    int currentY = current.y;

    
    double Move = OptimalMove(currentX, currentY);
    
    std::cout<<std::endl;
    std::cout<<std::endl;
    std::cout<<"/////////  UTILITY MAP    /////////"<<std::endl<<std::endl<<std::endl;
    for(int i=0; i < H; i++){
        for(int j = 0; j < W; j++){
            
            double Move = OptimalMove(i, j);
            std::string out = "?";
            
            if(Move==0){
                out = "^";
            }
            if(Move==1){
                out = ">";
            }
            if(Move==2){
                out = "v";
            }
            if(Move==3){
                out = "<";
            }
            
            
            std::cout<<out;
        }
        std::cout<<std::endl;
        
    }
    std::cout<<std::endl;
    std::cout<<std::endl;
    
    
    if(Move == 0){
         std::cout<<"UP "<<std::endl;
        return (RobotAction)(MOVE_UP);
    
    }if(Move == 1){
        std::cout<<"RIGHT "<<std::endl;
        return (RobotAction)(MOVE_RIGHT);
        
    }if(Move == 2){
        std::cout<<"DOWN "<<std::endl;
        return (RobotAction)(MOVE_DOWN);
        
    }
    //MOVE ==3
    std::cout<<"LEFT "<<std::endl;
    return (RobotAction)(MOVE_LEFT);
    
    
    
}
