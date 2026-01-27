
#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "frc2/command/SubsystemBase.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg;

    cfg.Slot0.kP = 0.1;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;

    cfg.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

    intake_driver.GetConfigurator().Apply(cfg);

    intake_driver.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);
  }
  void Periodic() override;

  void SetIntake(bool intake_set);

 private:
  // motors here
  bool intake_on = false;

  // falcon 500
  ctre::phoenix6::hardware::TalonFX intake_driver{40};

  ctre::phoenix6::controls::VelocityVoltage intake_driver_speed =
      ctre::phoenix6::controls::VelocityVoltage{5_tps}.WithSlot(0);

  ctre::phoenix6::controls::VelocityVoltage stop_speed =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
};
