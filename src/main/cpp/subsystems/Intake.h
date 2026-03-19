
#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/ClosedLoopTypes.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/config/SparkMaxConfig.h>

#include "../Regulator.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "frc2/command/SubsystemBase.h"
#include "rev/config/ClosedLoopConfig.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfigAccessor.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem() {
    rev::spark::SparkBaseConfig intake_config{};

    intake_config.encoder.PositionConversionFactor(1).VelocityConversionFactor(
        1);
    intake_config.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
        .Pid(0.1, 0, 0)
        .OutputRange(-1, 1);

    intake_driver.Configure(intake_config, rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);


    intake_up_down_config.encoder.PositionConversionFactor(1)
        .VelocityConversionFactor(1);
    intake_up_down_config.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
        .Pid(0.1, 0, 0)
        .OutputRange(-1, 1);

    intake_up_down_config.closedLoop.maxMotion.CruiseVelocity(0.001);
  
    intake_up_down_config.SetIdleMode(rev::spark::SparkBaseConfig::kCoast);

    intake_up_down_driver.Configure(intake_up_down_config,
                                    rev::ResetMode::kResetSafeParameters,
                                    rev::PersistMode::kPersistParameters);

    up_down_regulator.Zero();
    up_down_regulator.SetTargets(0, -6);
  }
  void Periodic() override;

  void SetIntake(bool intake_set);

  enum IntakeUpDown { INTAKE_UP, INTAKE_DOWN, INTAKE_NONE};
  enum IntakeDischarge { INTAKE_SUCK, INTAKE_REPEL };
  void SetIntakeUpDown(IntakeUpDown c_intake_up_down);
  void SetIntakeDirection(IntakeDischarge intake_set);

 private:
    rev::spark::SparkBaseConfig intake_up_down_config{};
  // motors here
  bool intake_on = false;
  IntakeDischarge intake_dir = INTAKE_SUCK;
  IntakeUpDown intake_up_down = INTAKE_NONE;

  rev::spark::SparkMax intake_driver =
      rev::spark::SparkMax(40, rev::spark::SparkMax::MotorType::kBrushed);
  rev::spark::SparkClosedLoopController intake_driver_controller =
      intake_driver.GetClosedLoopController();

  rev::spark::SparkMax intake_up_down_driver =
      rev::spark::SparkMax(41, rev::spark::SparkMax::MotorType::kBrushless);
  //
  rev::spark::SparkClosedLoopController intake_up_down_controller =
      intake_up_down_driver.GetClosedLoopController();
  //
  rev::spark::SparkRelativeEncoder intake_up_down_encoder =
      intake_up_down_driver.GetEncoder();
  //
  MotorRegulator up_down_regulator =
      MotorRegulator(&intake_up_down_driver, &intake_up_down_controller);
};
