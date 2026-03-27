#include "Agitate.h"

void AgitateSubsystem::Periodic() {
  // JULIA: Perodic function
  if (agitate_on == 1) {
    if (agitate_up) {
      agitate_driver_controller.SetSetpoint(
          0.4, 
          rev::spark::SparkLowLevel::ControlType::kDutyCycle);
    }else{

      agitate_driver_controller.SetSetpoint(
          -0.4, 
          rev::spark::SparkLowLevel::ControlType::kDutyCycle);
    }
  } else if (agitate_on == 0){
    agitate_driver_controller.SetSetpoint(
        0, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
  }
}

void AgitateSubsystem::SetAgitate(uint16_t  agitate_set) { agitate_on = agitate_set; }
