// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>

#include <cmath>
#include <ctre/phoenix6/Pigeon2.hpp>

#include "../Constants.h"
#include "MAXSwerveModule.h"
#include "Shooter.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem( bool *m_setDist, float *dist ) ;

  bool *m_setDist ;
  float *dist ;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   * @param slowMode      Whether the max speed or slow mode speed will be used
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, bool slowMode);

  frc::ChassisSpeeds m_chassisSpeeds;

  void driveRobotRelative(frc::ChassisSpeeds speeds) {
    m_chassisSpeeds = speeds;
    auto states = kDriveKinematics.ToSwerveModuleStates(speeds);

    kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    m_frontLeft.SetDesiredState(fl);
    m_frontRight.SetDesiredState(fr);
    m_rearLeft.SetDesiredState(bl);
    m_rearRight.SetDesiredState(br);
  }

  frc::ChassisSpeeds getChassisSpeeds(void) { return m_chassisSpeeds; }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  std::pair<frc::Pose2d, double> GetHubDistance() {
    if (m_odometry.GetEstimatedPosition().X().value() > 4.5 &&
        m_odometry.GetEstimatedPosition().X().value() < 12.1) {

      return {m_odometry.GetEstimatedPosition().Nearest(
          {this->blueSideBlow1, this->blueSideBlow2, 
          this->redSideBlow1, this->redSideBlow2}), 0};
    } else {
      return {m_odometry.GetEstimatedPosition().Nearest(
          {this->blueSideHub, this->redSideHub}), 5};
    }
  };

  double MyHomiePythagoras() {
    double AValTriangle = GetHubDistance().first.X().value() -
                          m_odometry.GetEstimatedPosition().X().value();
    double BvalTriangle = GetHubDistance().first.Y().value() -
                          m_odometry.GetEstimatedPosition().Y().value();
    return std::sqrt(std::pow(AValTriangle, 2) + std::pow(BvalTriangle, 2));
  }

  double GetHubRelRot() {
    return std::atan2(GetHubDistance().first.X().value() -
                          m_odometry.GetEstimatedPosition().X().value(),
                      GetHubDistance().first.Y().value() -
                          m_odometry.GetEstimatedPosition().Y().value());
  }
  std::pair<double, double> GetHubRelDist() {
    return {GetHubDistance().first.X().value() -
                          m_odometry.GetEstimatedPosition().X().value(),
                      GetHubDistance().first.Y().value() -
                          m_odometry.GetEstimatedPosition().Y().value()};
  }

  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2}};

  ctre::phoenix6::hardware::Pigeon2 m_pigeon{60};

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearRight;

  // The gyro sensor

  // Odometry class for tracking robot pose
  // 4 defines the number of modules
  frc::SwerveDrivePoseEstimator<4> m_odometry;
  frc::Field2d m_field;
  frc::Field2d m_hub_field;


  // OLIVER + JULIA: Change this with the ball target positions
  // frc::Pose2d hi = frc::Pose2d(2_m, 2_m, frc::Rotation2d(0_deg));

  // dean stop pulling up pathplanner
  // one side two side red side blue side

  frc::Pose2d redSideHub =
      frc::Pose2d(11.908_m, 4.037_m, frc::Rotation2d(0_deg));
  frc::Pose2d blueSideHub =
      frc::Pose2d(4.632_m, 4.037_m, frc::Rotation2d(0_deg));

  frc::Pose2d redSideBlow1 =
      frc::Pose2d(14.400_m, 6.400_m, frc::Rotation2d(0_deg));
  frc::Pose2d redSideBlow2 =
      frc::Pose2d(14.400_m, 2_m, frc::Rotation2d(0_deg));

  frc::Pose2d blueSideBlow1 =
      frc::Pose2d(2_m, 6.400_m, frc::Rotation2d(0_deg));
  frc::Pose2d blueSideBlow2 =
      frc::Pose2d(2_m, 2_m, frc::Rotation2d(0_deg));


  // this is the function to get the nearest pose
  // m_odometry.GetEstimatedPosition().Nearest({this->hi, this->hi});
};
