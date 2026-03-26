#include "Shooter.h"

#include <cstddef>
#include <string>

#include "frc/smartdashboard/SmartDashboard.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"

void ShooterSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Shooter Target Velocity",
                                 traj.get_velocity());
  frc::SmartDashboard::PutNumber(
      "Shooter Real Velocity",
      (shooter_driver.GetVelocity()
           .GetValue()
           .convert<units::angular_velocity::radians_per_second>() /
       wheel_radius)
          .value());

  frc::SmartDashboard::PutNumber("Shooter Pitch", traj.get_pitch());
  frc::SmartDashboard::PutBoolean("Shooter On", shooter_on);
  frc::SmartDashboard::PutBoolean("Shooter Index On", shooter_intake_on);
  frc::SmartDashboard::PutString("Manual Shooter Speed", std::to_string(shooter_manual_speed) + " tps");

  // ME: so this will get the wheel spinning at the target velocity off the ball
  // there will be a conversion loss so this needs to be tweaked
  shooter_driver_speed.WithVelocity(1_rad_per_s * wheel_radius *
                                    traj.get_velocity() * 4.2);

  if (shooter_on) {
    shooter_driver.SetControl(shooter_driver_speed);

    // JULIA this is the right syntax! just replace this with your new motor
    // shooter_turner.SetControl(turn_position);

  } else {
    shooter_driver.SetControl(stop_speed);
  }

  if (shooter_intake_on) {
    shooter_intake_driver_controller.SetSetpoint(
        0.4, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
  } else {
    shooter_intake_driver_controller.SetSetpoint(
        0, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
  }

  double servo_location = std::clamp((units::radian_t(traj.get_pitch()).convert<units::degree>().value() - 45.0)/(78.0-45.0), 0.0, 1.0);
  
  frc::SmartDashboard::PutNumber("Shooter Servo Pitch", servo_location);
  servo_left.Set(servo_location);
  servo_right.Set(servo_location);
}

void ShooterSubsystem::SetShooter(bool shooter_set) {
  shooter_on = shooter_set;
}
void ShooterSubsystem::SetShooterIntake(bool shooter_set) {
  shooter_intake_on = shooter_set;
}

void ShooterSubsystem::SetShooterMode(ShooterMode mode_set) { mode = mode_set; }

// JULIA: units::turns argument here to this function just like you do in
// the Shooter.h
void ShooterSubsystem::SetShooterRot(units::angle::turn_t turns) {
  // JULIA: shooter_turner -> turn_shooter
  //
  // JULIA: also! pass the turns argument into WithPosition
  turn_position.WithPosition(turns);
};

void ShooterSubsystem::SetHubDistance(units::meter_t distance, double height) {
  for (size_t i = 0; i < 10; i++) {
    // ME: adjust values here too for accuracy
    traj.calculate_loss(distance.convert<units::length::feet>().value(),
                        height);
  }
}
