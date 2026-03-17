
#pragma once

#include "TrajectoryCalculator.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "frc2/command/SubsystemBase.h"
#include "frc/Servo.h"
#include "rev/SparkMax.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"

enum ShooterMode {
  SHOOTER_NONE,
  SHOOTER_MANUAL,
  SHOOTER_AUTO,
};

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg;

    cfg.Slot0.kS =
        0.1;  // To account for friction, add 0.1 V of static feedforward
    cfg.Slot0.kV =
        0.30;  // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V,
               // 1/8.33 = 0.12 volts / rotation per second
    cfg.Slot0.kP =
        0.11;  // An error of 1 rotation per second results in 0.11 V output
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;

    cfg.Voltage.PeakForwardVoltage = 12_V;
    cfg.Voltage.PeakReverseVoltage = -12_V;

    ctre::phoenix6::configs::TalonFXConfiguration cfg2;

    cfg2.Slot0.kS =
        0.1;  // To account for friction, add 0.1 V of static feedforward
    cfg2.Slot0.kV =
        0.4;  // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V,
              // 1/8.33 = 0.12 volts / rotation per second
    cfg2.Slot0.kP =
        0.3;  // An error of 1 rotation per second results in 0.11 V output
    cfg2.Slot0.kI = 0.0;
    cfg2.Slot0.kD = 0.0;

    cfg2.Voltage.PeakForwardVoltage = 12_V;
    cfg2.Voltage.PeakReverseVoltage = -12_V;

    cfg.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    cfg2.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

    shooter_driver.GetConfigurator().Apply(cfg);

    shooter_driver_complement.GetConfigurator().Apply(cfg2);

    shooter_driver.SetControl(stop_speed);

    shooter_driver_complement.SetControl(ctre::phoenix6::controls::Follower{
        shooter_driver.GetDeviceID(),
        ctre::phoenix6::signals::MotorAlignmentValue::Opposed});

    // shooter_turner.GetConfigurator().Apply(cfg);

    // JULIA: config needs to be set here like above just with
    // shooter_turner

    shooter_driver.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);
    shooter_driver_complement.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);

    // shooter_turner.SetNeutralMode(
    //     ctre::phoenix6::signals::NeutralModeValue::Brake);

    // JULIA: any new motor you add will need to be configurated here
    // this is the constructor so any code here will be run as soon as the
    // object is instanced (ie the robot starts)
    //

    rev::spark::SparkBaseConfig shooter_intake_config{};

    shooter_intake_config.encoder.PositionConversionFactor(1)
        .VelocityConversionFactor(1);
    shooter_intake_config.closedLoop
        .Pid(0.1, 0, 0)
        .OutputRange(-1, 1);
    shooter_intake_config.Inverted(true);

    shooter_intake_driver.Configure(shooter_intake_config,
                                    rev::ResetMode::kResetSafeParameters,
                                    rev::PersistMode::kPersistParameters);

    // traj.set_fixed(
    //     units::angle::degree_t{65}.convert<units::angle::radian>().value());
  }
  void Periodic() override;

  void SetShooter(bool shooter_set);
  void SetShooterIntake(bool shooter_set);
  void SetShooterMode(ShooterMode mode);

  // JULIA: add a argument with units::turns a argument
  void SetShooterRot(units::angle::turn_t turns);

  void SetHubDistance(units::meter_t distance, double height);
  
  bool shooter_automatic_on = true;
  float shooter_manual_speed = 80;

 private:
  bool shooter_on = false;
  bool shooter_intake_on = false;

  ShooterMode mode = SHOOTER_NONE;

  // falcon 500
  ctre::phoenix6::hardware::TalonFX shooter_driver{20};
  ctre::phoenix6::hardware::TalonFX shooter_driver_complement{21};
  // ctre::phoenix6::hardware::TalonFX shooter_turner{21};
  // JULIA: we will need another motor so make a new
  // ctre::phoenix6::hardware::TalonFX
  //
  // refer to the bible when picking IDs
  // https://docs.google.com/document/d/1VkR9zvviwuhPBft1adSYg7TGN60f-zLs2Nebqqzaj-k/edit?usp=sharing

  const float wheel_radius = 2;
  ctre::phoenix6::controls::VelocityVoltage shooter_driver_speed =
      ctre::phoenix6::controls::VelocityVoltage{0_rad_per_s * wheel_radius}
          // ctre::phoenix6::controls::VelocityVoltage{20_tps}
          // TODO: update to match gear ratio
          .WithSlot(0);

  ctre::phoenix6::controls::VelocityVoltage stop_speed =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

  // JULIA: this is the info we give to a ctre::phoenix6::hardware::TalonFX to
  // set the speed of the motor
  //
  // so _tps here is turns per second, to set postion we need _tr ie turns
  //
  // WithPosition is a function to update the position later on so not during
  // intialization (intialization is here) we should have a SetShooterRot() or
  // something function that will update the rot
  // ctre::phoenix6::controls::PositionDutyCycle turn_shooter =
  //     ctre::phoenix6::controls::PositionDutyCycle{0.2_tps}.WithPosition();

  /* JULIA: this is the info we give to a ctre::phoenix6::hardware::TalonFX to
   set the speed of the motor

   so _tps here is turns per second, to set postion we need _tr ie turns

   WithPosition is a function to update the position later on so not during
   intialization (intialization is here) we should have a SetShooterRot() or
   something function that will update the rot */
  ctre::phoenix6::controls::PositionDutyCycle turn_position =
      ctre::phoenix6::controls::PositionDutyCycle{1_rad};

  rev::spark::SparkMax shooter_intake_driver =
      rev::spark::SparkMax(22, rev::spark::SparkMax::MotorType::kBrushless);
  rev::spark::SparkClosedLoopController shooter_intake_driver_controller =
      shooter_intake_driver.GetClosedLoopController();

  frc::Servo servo_left{0};
  frc::Servo servo_right{1};

  TrajectoryCalculator traj;
};
