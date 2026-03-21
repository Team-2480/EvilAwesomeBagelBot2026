#include "Agitate.h"

void AgitateSubsystem::Periodic() {
  // JULIA: Perodic function
  if (agitate_on) {
    agitate_driver_controller.SetSetpoint(
        0.4, 
        rev::spark::SparkLowLevel::ControlType::kDutyCycle);
  } else {
    agitate_driver_controller.SetSetpoint(
        0, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
  }
}

void AgitateSubsystem::SetAgitate(bool agitate_set) { agitate_on = agitate_set; }
