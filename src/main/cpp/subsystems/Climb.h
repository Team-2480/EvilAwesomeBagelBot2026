
#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/PositionDutyCycle.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include "frc2/command/SubsystemBase.h"

class ClimbSubsystem : public frc2::SubsystemBase {
 public:
  ClimbSubsystem() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg;

    cfg.Slot0.kP = 0.1;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;

    cfg.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    climb_spool.GetConfigurator().Apply(cfg);

    climb_spool.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);
    climb_spool.SetPosition(0_tr);

    // down_pos.WithFeedForward(1_V);
    // up_pos.WithFeedForward(1_V);
  }
  void Periodic() override;

  void SetClimb(bool climb_set);

  bool climb_up = true;  // robot starts in up

 private:
  ctre::phoenix6::hardware::TalonFX climb_spool{30};

  ctre::phoenix6::controls::PositionVoltage up_pos =  // we start up
      ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);

  ctre::phoenix6::controls::PositionVoltage down_pos =
      ctre::phoenix6::controls::PositionVoltage{36_tr * 2}.WithSlot(0);
};
