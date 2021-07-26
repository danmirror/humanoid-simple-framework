/*
 * desc : rscuad move joint
 * year : 2021
 * dev  : danu andrean
 *
 */

#include "rscuad_manager/rscuad_manager.h"


int main(int argc, char **argv)
{
    // alocation memory
    rscuad::rscuad_manager *rscuad = new rscuad::rscuad_manager;

    //initial power
    rscuad->manager_init();

   while(1){
      /* set joint want to move
      *  
      *  robot    -> just hardware robot
      *  gazebo   -> just simulation using gazebo
      *  all      -> gazebo and robot
      *
      */
      rscuad->move_joint("robot",18,190);
   }

    
}