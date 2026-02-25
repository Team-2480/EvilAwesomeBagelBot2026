#include "Intake.h"

#include <cstdio>

void IntakeSubsystem::Periodic() {
  // send motor info to intake motor
  if (intake_on) {
    intake_driver_controller.SetSetpoint(
        intake_dir == INTAKE_SUCK ? -1 : 1, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
  } else {
    intake_driver_controller.SetSetpoint(
        0, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
  }

  // printf("curently at %f\n",
  // intake_up_down_driver.GetEncoder().GetPosition());
  //
  // switch (intake_up_down) {
  //   case INTAKE_UP:
  //     up_down_regulator.Up();
  //     break;
  //   case INTAKE_DOWN:
  //     up_down_regulator.Down();
  //     break;
  // }
}

void IntakeSubsystem::SetIntake(bool intake_set) { intake_on = intake_set; }
void IntakeSubsystem::SetIntakeDirection(IntakeDischarge intake_set) { intake_dir = intake_set; }

void IntakeSubsystem::SetIntakeUpDown(IntakeUpDown c_intake_up_down) {
  intake_up_down = c_intake_up_down;
}
