#include "Shooter.h"

void ShooterSubsystem::Periodic() {
  switch (mode) {
    case SHOOTER_AUTO:
      break;
    case SHOOTER_MANUAL:
      break;
    case SHOOTER_NONE:
      break;
  }

  if (shooter_on) {
    shooter_driver.SetControl(shooter_driver_speed);

    // JULIA this is the right syntax! just replace this with your new motor
    shooter_turner.SetControl(turn_position);

  } else {
    shooter_driver.SetControl(stop_speed);
  }
}

void ShooterSubsystem::SetShooter(bool shooter_set) {
  shooter_on = shooter_set;
}

void ShooterSubsystem::SetShooterMode(ShooterMode mode_set) { mode = mode_set; }

// JULIA(NEW!): units::turns argument here to this function just like you do in
// the Shooter.h
void ShooterSubsystem::SetShooterRot(units::angle::turn_t turns) {
  // JULIA(NEW!): shooter_turner -> turn_shooter
  //
  // JULIA(NEW!): also! pass the turns argument into WithPosition
  turn_position.WithPosition(turns);
};
