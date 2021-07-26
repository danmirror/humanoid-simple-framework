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
      //set joint
      rscuad->move_joint("robot",18,190);
   }

    
}