// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <cstdio>
#include <memory>
#include <utility>

#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "pid.cpp"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Intake.h"

using namespace DriveConstants;
using namespace pathplanner;

Robot::Robot() {
  // Initialize all of your commands and subsystems here

  m_chooser.AddOption("No Auto", AUTO_NOTHING);
  // m_chooser.AddOption("Blue Default", AUTO_BLUE_DEFAULT);
  // m_chooser.AddOption("Red Default", AUTO_RED_DEFAULT);
  //
  frc::SmartDashboard::PutData("Auto Selection", &m_chooser);

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  units::radians_per_second_t appliedRot = units::radians_per_second_t{0};
  auto autoRot = units::radians_per_second_t{rot_pid.calculate(
      0, m_drive.GetHubRelRot() - m_drive.GetHeading().value())};

  frc::SmartDashboard::PutNumber("Auto Rotation", autoRot.value());

  if (m_findRot) {
    appliedRot = autoRot;
  } else {
    appliedRot = -units::radians_per_second_t{frc::ApplyDeadband(
        std::pow(m_driveController.GetZ(), 3), OIConstants::kDriveDeadband)};
  }

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this, appliedRot] {
        m_drive.Drive(-units::meters_per_second_t{frc::ApplyDeadband(
                          std::pow(m_driveController.GetY(), 3),
                          OIConstants::kDriveDeadband)},
                      -units::meters_per_second_t{frc::ApplyDeadband(
                          std::pow(m_driveController.GetX(), 3),
                          OIConstants::kDriveDeadband)},
                      appliedRot, m_globalLocal, m_slowMode);
      },
      {&m_drive}));
}

void Robot::ConfigureButtonBindings() {
  // button to stop being pushed
  frc2::JoystickButton(&m_driveController, 6)  // button 6 on joystick?
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

  frc2::JoystickButton(&m_driveController, 5)  // button 6 on joystick?
      .ToggleOnTrue(new frc2::RunCommand(
          [this] {
            m_globalLocal = !m_globalLocal;
            frc::SmartDashboard::PutBoolean("Global Local", m_globalLocal);
          },
          {&m_drive}));

  // slow mode
  frc2::JoystickButton(&m_driveController, 1)  // trigger
      .ToggleOnTrue(
          new frc2::InstantCommand([this]() { m_slowMode = true; }, {}));

  frc2::JoystickButton(&m_driveController, 1)  // trigger
      .ToggleOnFalse(
          new frc2::InstantCommand([this]() { m_slowMode = false; }, {}));

  // find rot
  frc2::JoystickButton(&m_driveController, 2)  // trigger
      .ToggleOnTrue(
          new frc2::InstantCommand([this]() { m_findRot = true; }, {}));

  frc2::JoystickButton(&m_driveController, 2)  // trigger
      .ToggleOnFalse(
          new frc2::InstantCommand([this]() { m_findRot = false; }, {}));

  // activate intake
  // frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kA)
  //     .ToggleOnTrue(new frc2::InstantCommand(
  //         [this] { m_intake.SetIntake(true); }, {&m_intake}));
  // frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kA)
  //     .ToggleOnFalse(new frc2::InstantCommand(
  //         [this] { m_intake.SetIntake(false); }, {&m_intake}));

  // climb
  frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kB)
      .ToggleOnTrue(new frc2::InstantCommand(
          [this] { m_climb.SetClimb(false); }, {&m_climb}));
  frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kB)
      .ToggleOnFalse(new frc2::InstantCommand(
          [this] { m_climb.SetClimb(true); }, {&m_climb}));

  // OLIVER: see this:
  auto climb_true =
      new frc2::InstantCommand([this] { m_climb.SetClimb(true); }, {&m_climb});
  auto shared_climb_true = std::shared_ptr<frc2::Command>(climb_true);
  pathplanner::NamedCommands::registerCommand("climbTrue",
                                              std::move(shared_climb_true));

  // shooter
  frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kA)
      .ToggleOnTrue(new frc2::InstantCommand(
          [this] { m_shooter.SetShooter(true); }, {&m_shooter}));
  frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kA)
      .ToggleOnFalse(new frc2::InstantCommand(
          [this] { m_shooter.SetShooter(false); }, {&m_shooter}));

  // intake
  frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kX)
      .ToggleOnTrue(new frc2::InstantCommand(
          [this] { m_intake.SetIntake(true); }, {&m_intake}));
  frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kX)
      .ToggleOnFalse(new frc2::InstantCommand(
          [this] { m_intake.SetIntake(false); }, {&m_intake}));

  frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kY)
      .ToggleOnTrue(new frc2::InstantCommand(
          [this] { m_intake.SetIntakeDirection(IntakeSubsystem::INTAKE_BLOW); },
          {&m_intake}));
  frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kY)
      .ToggleOnFalse(new frc2::InstantCommand(
          [this] { m_intake.SetIntakeDirection(IntakeSubsystem::INTAKE_SUCK); },
          {&m_intake}));
}

frc2::CommandPtr Robot::GetAutonomousCommand() {
  // had to delete the other method for auto cause it broke

  switch (m_chooser.GetSelected()) {}
  return PathPlannerAuto("Yo").ToPtr();
}
