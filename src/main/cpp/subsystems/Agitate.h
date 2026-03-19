#pragma once

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include "frc2/command/SubsystemBase.h"

// JULIA (NEW!): Header files!

class AgitateSubsystem : public frc2::SubsystemBase {
 public:
  AgitateSubsystem() {
    // Header issue!

    rev::spark::SparkBaseConfig agitate_config{};

    agitate_config.encoder.PositionConversionFactor(1).VelocityConversionFactor(
        1);

    agitate_config.closedLoop
        .SetFeedbackSensor(
            // Header issue!
            rev::spark::FeedbackSensor::kPrimaryEncoder)
        .Pid(0.1, 0, 0)
        .OutputRange(-1, 1);

    agitate_driver.Configure(agitate_config,
                             rev::ResetMode::kResetSafeParameters,
                             rev::PersistMode::kPersistParameters);

    // JULIA (NEW!): Missing closing curly brace here
  }

  void Periodic() override;

  void SetAgitate(bool agitate_set);
  bool agitate_on = false;

  // JULIA (NEW!) functions to change turn speed on off

 private:
  rev::spark::SparkMax agitate_driver =
      rev::spark::SparkMax(50, rev::spark::SparkMax::MotorType::kBrushed);
  // Header issue!
  rev::spark::SparkClosedLoopController agitate_driver_controller =
      agitate_driver.GetClosedLoopController();

  // JULIA(NEW!): Erroneous squirely brace
};
