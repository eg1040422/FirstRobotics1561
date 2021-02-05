// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Timer.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include "ctre/Phoenix.h"
#include <frc/GenericHID.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/VictorSP.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SetShooters(float);
  void SetDrive(float,float);
  void SetIntake(float);
  void SwitchThings(int,bool);
  
 private:
  frc::XboxController DriverJoystick{0};
  frc::XboxController SecondController{1};
  frc::VictorSP IntakeArm{0};
  frc::VictorSP IntakeLeft{1};
  frc::VictorSP IntakeRight{2};
  frc::Spark LiftMotor{3};
  TalonSRX DriveLeft1{5};
  TalonSRX DriveLeft2{6};
  TalonSRX DriveRight1{7};
  TalonSRX DriveRight2{8};
  TalonSRX ShooterTop{9};
  TalonSRX ShooterBottom{10};
  frc::Solenoid ArmExtend{3};
  frc::Solenoid ArmRetract{2};
  frc::Solenoid ShooterUp{1};
  frc::Solenoid ShooterDown{0};
  frc::Solenoid ClimbLock{4};
  frc::Solenoid ClimbRelease{5};
  frc::Timer Timer;
  nt::NetworkTableEntry xEntry;
  nt::NetworkTableEntry yEntry;
  double LeftDriveSpeed;
  double RightDriveSpeed;
  double Deadband;
  bool ShooterHigh;
};
