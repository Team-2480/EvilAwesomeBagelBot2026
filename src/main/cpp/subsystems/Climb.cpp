#include "Climb.h"

#include "frc/smartdashboard/SmartDashboard.h"

void ClimbSubsystem::Periodic() {
  if (climb_up) {
    climb_spool.SetControl(up_pos);
  } else {
    climb_spool.SetControl(down_pos);
  }
  frc::SmartDashboard::PutBoolean("Climb Up", climb_up);
  frc::SmartDashboard::PutNumber("Climb Motor Pos", climb_spool.GetPosition().GetValue().value());
}

void ClimbSubsystem::SetClimb(bool climb_set) { climb_up = climb_set; }
