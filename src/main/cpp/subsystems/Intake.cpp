#include "Intake.h"

#include <cstdio>

void IntakeSubsystem::Periodic() {
  // send motor info to intake motor
  if (intake_on) {
    intake_driver.SetControl(intake_driver_speed);
  } else {
    intake_driver.SetControl(stop_speed);
  }
}

void IntakeSubsystem::SetIntake(bool intake_set) { intake_on = intake_set; }
