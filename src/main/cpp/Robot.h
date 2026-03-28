// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include <string>

#include "Constants.h"
#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "pid.h"
#include "subsystems/Climb.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Intake.h"
#include "subsystems/Agitate.h"
#include "subsystems/Shooter.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class Robot {
 public:
  Robot();

  pathplanner::PathPlannerAuto * GetAutonomousCommand();

 private:
  // The driver's controller
  frc::Joystick m_driveController{0};
  frc::XboxController m_actionController{1};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  bool m_slowMode = false;
  bool m_globalLocal = true;
  bool m_findRot = false;
  bool m_setDist = false;
  float dist = 5.0;

  DriveSubsystem m_drive = DriveSubsystem(&m_shooter, &m_setDist, &dist);

  // IntakeSubsystem m_intake;
  ClimbSubsystem m_climb;
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  AgitateSubsystem m_agitate;

  // The chooser for the autonomous routines
  std::vector<std::string> auto_names{"Nothing", "StraightClimb", "RedRPlow", "RedLPlow", "RedLShoot", "RedRShoot", "RedMidShoot", "RedMidShootClimb", "RedRShootClimb", "RedLShootClimb", "BlueLPlow", "BlueRPlow", "BlueLShoot", "BlueRShoot", "BlueMidShoot", "BlueMidShootClimb", "BlueRShootClimb", "BlueLShootClimb"};
  frc::SendableChooser<std::string> m_chooser;

  PID rot_pid = PID(1.0/50.0, 10, -10, 0.1, 0, 0);

  void ConfigureButtonBindings();
};
