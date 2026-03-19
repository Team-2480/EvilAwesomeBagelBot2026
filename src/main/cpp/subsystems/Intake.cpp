#include "Intake.h"

#include <cstdio>

#include "frc/smartdashboard/SmartDashboard.h"

void IntakeSubsystem::Periodic() {
  // send motor info to intake motor
  if (intake_on) {
    intake_driver_controller.SetSetpoint(
        intake_dir == INTAKE_SUCK ? -0.5 : 0.5,
        rev::spark::SparkLowLevel::ControlType::kDutyCycle);
  } else {
    intake_driver_controller.SetSetpoint(
        0, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
  }

  frc::SmartDashboard::PutNumber(
      "Intake Up Down Position",
      intake_up_down_driver.GetEncoder().GetPosition());

  switch (intake_up_down) {
    case INTAKE_UP:
      break;
    case INTAKE_DOWN:
      break;
    default:
      break;
  }
}

void IntakeSubsystem::SetIntake(bool intake_set) { intake_on = intake_set; }
void IntakeSubsystem::SetIntakeDirection(IntakeDischarge intake_set) {
  intake_dir = intake_set;
}

void IntakeSubsystem::SetIntakeUpDown(IntakeUpDown c_intake_up_down) {
  intake_up_down = c_intake_up_down;
  if (c_intake_up_down == INTAKE_UP) {
    intake_up_down_config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
    up_down_regulator.Up();
  } else if (c_intake_up_down == INTAKE_DOWN) {
    intake_up_down_config.SetIdleMode(rev::spark::SparkBaseConfig::kCoast);
    up_down_regulator.Down();
  }
  intake_up_down_driver.Configure(intake_up_down_config,
                                  rev::ResetMode::kResetSafeParameters,
                                  rev::PersistMode::kPersistParameters);
}
