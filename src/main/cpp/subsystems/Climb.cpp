#include "Climb.h"


void ClimbSubsystem::Periodic() {
    //send motor info to climb motor
}
void ClimbSubsystem::SetClimb (bool climb_set) { climb_up = climb_set; }

//No clue what is happening on top :p
//No clue whats happening under either 

if (Get.button) {
    //start motor, but only run as much as needed?
    std::cout <<"climb motor activated";

} else {
    climb_set = false;
}

