#include "Climb.h"

void ClimbSubsystem::Periodic() {
  if (climb_up) {
    climb_spool.SetControl(up_pos);
  } else {
    climb_spool.SetControl(down_pos);
  }
}

void ClimbSubsystem::SetClimb(bool climb_set) { climb_up = climb_set; }
