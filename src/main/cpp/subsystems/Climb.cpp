#include "Climb.h"
#include <iostream>


void ClimbSubsystem::Periodic() {
    if (climb_up) {
    //start motor, but only run as much as needed
    std::cout <<"climb motor activated";
}
}
void ClimbSubsystem::SetClimb (bool climb_set) { climb_up = climb_set; }

//No clue what is happening on top :p

