// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit()
{
  LeftDriveSpeed = 0;
  RightDriveSpeed = 0;
  Deadband = .15;
  ShooterHigh = true;
}
void Robot::RobotPeriodic()
{
  
}
void Robot::AutonomousInit()
{
  Timer.Reset();
  Timer.Start();
}
void Robot::AutonomousPeriodic()
{
  ShooterUp.Set(true);
  ShooterDown.Set(false);
  ShooterHigh = true;
  if (Timer.Get() < 2.0)
  {
    ShooterTop.Set(ControlMode::PercentOutput, 0.35);
    ShooterBottom.Set(ControlMode::PercentOutput,0.35); // spin up shooter
  } 
  else if(Timer.Get()< 10.0)
  {
    IntakeLeft.Set(ControlMode::PercentOutput,0.2);
    IntakeRight.Set(ControlMode::PercentOutput,-0.2); //feed balls to shooter
  }
  else if(Timer.Get() < 13)
  {
    DriveLeft1.Set(ControlMode::PercentOutput,.2);
    DriveLeft2.Set(ControlMode::PercentOutput,.2);
    DriveRight1.Set(ControlMode::PercentOutput,-.2);
    DriveRight2.Set(ControlMode::PercentOutput,-.2);
  }
  else
  {
    DriveLeft1.Set(ControlMode::PercentOutput,0);
    DriveLeft2.Set(ControlMode::PercentOutput,0);
    DriveRight1.Set(ControlMode::PercentOutput,0);
    DriveRight2.Set(ControlMode::PercentOutput,0);
    IntakeLeft.Set(ControlMode::PercentOutput,0);
    IntakeRight.Set(ControlMode::PercentOutput,0);
    ShooterTop.Set(ControlMode::PercentOutput, 0);
    ShooterBottom.Set(ControlMode::PercentOutput,0);
  }
}
void Robot::TeleopInit()
{
  ShooterTop.ConfigFactoryDefault();
  ShooterTop.SetInverted(false);
  ShooterTop.SetSensorPhase(false);
}
void Robot::TeleopPeriodic()
{
  auto Inst = nt::NetworkTableInstance::GetDefault();
  auto Table = Inst.GetTable("limelight");
  xEntry = Table->GetEntry("X");
  yEntry = Table->GetEntry("Y");
  nt::NetworkTableEntry tx = Table->GetEntry("tx");
  nt::NetworkTableEntry ty = Table->GetEntry("ty");
  nt::NetworkTableEntry ta= Table->GetEntry("ta");
  nt::NetworkTableEntry tv = Table->GetEntry("tv");
  double X = tx.GetDouble(0.0);
  double Y = ty.GetDouble(0.0);
  double Area = ta.GetDouble(0.0);
  double Validtarget = tv.GetDouble(0.0);
  frc::SmartDashboard::PutNumber("LimeLightValidTarget", Validtarget);
  frc::SmartDashboard::PutNumber("LimelightX", X);
  frc::SmartDashboard::PutNumber("LimelightY", Y);
  frc::SmartDashboard::PutNumber("LimelightArea", Area);
  frc::SmartDashboard::PutBoolean("Locked", ClimbLock.Get());
  float Kp = 0.02f;
  float Kp2 = 0.0005f;
  //Shooter - Left Bumper
  if(DriverJoystick.GetBumper(frc::GenericHID::JoystickHand::kLeftHand))
  {
    frc::SmartDashboard::PutNumber("ShooterBottomVel", ShooterBottom.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("ShooterTopVel", ShooterTop.GetSelectedSensorVelocity());
    if(Validtarget == 1)
    {
      double adjust = (Kp *  tx.GetDouble(0.0));
      if(adjust <= 0){adjust = adjust - 0.1;}
      else{adjust = adjust + 0.1;}
      frc::SmartDashboard::PutNumber("adjust", adjust);
      DriveLeft1.Set(ControlMode::PercentOutput, adjust);
      DriveLeft2.Set(ControlMode::PercentOutput, adjust);
      DriveRight1.Set(ControlMode::PercentOutput, adjust);
      DriveRight2.Set(ControlMode::PercentOutput, adjust);
    }
    //following code came commented out. could be useful. idk.
    /*double desiredVel = 0.0;
    double shootingPowerBottom = 0.0;
    double shootingPowerTop = 0.0;
    double shooterVelBottom = ShooterBottom.getSelectedSensorVelocity();
    double shooterVelTop = ShooterTop.getSelectedSensorVelocity();
    if(ShooterHigh)
    {
    shootingPowerBottom =  shootingPowerBottom + (Kp2 * (desiredVel - shooterVelBottom));
    shootingPowerTop =  shootingPowerTop + (Kp2 * (desiredVel - shooterVelTop));
    frc::SmartDashboard::putNumber("shootingPower", shootingPowerTop);
    ShooterTop.set(ControlMode.PercentOutput, shootingPowerTop);
    ShooterBottom.set(ControlMode.PercentOutput, shootingPowerBottom);
    }
    else if(DriverJoystick.getBumper(frc::GenericHID::JoystickHand::kRightHand))
    {
    ShooterTop.set(ControlMode.PercentOutput, shootingPowerTop);
    ShooterBottom.set(ControlMode.PercentOutput, shootingPowerBottom);
    }
    else
    {
    ShooterTop.set(ControlMode.PercentOutput, shootingPowerTop);
    ShooterBottom.set(ControlMode.PercentOutput, shootingPowerBottom);
    }*/
    if(ShooterHigh)
    {
    ShooterTop.Set(ControlMode::PercentOutput, 0.35);
    ShooterBottom.Set(ControlMode::PercentOutput,0.35);
    }
    else if(DriverJoystick.GetBumper(frc::GenericHID::JoystickHand::kRightHand))
    {
    ShooterTop.Set(ControlMode::PercentOutput, 0.55);
    ShooterBottom.Set(ControlMode::PercentOutput,0.55);
    }
    else
    {
    ShooterTop.Set(ControlMode::PercentOutput, 0.45);
    ShooterBottom.Set(ControlMode::PercentOutput,0.45);
    }
  }
  else
  {
    ShooterTop.Set(ControlMode::PercentOutput, 0);
    ShooterBottom.Set(ControlMode::PercentOutput,0);
  }
  //Conveyor Control
  if(DriverJoystick.GetRawButton(1))
  {
    IntakeLeft.Set(ControlMode::PercentOutput,0.3);
    IntakeRight.Set(ControlMode::PercentOutput,-0.3);
  }
  else if(DriverJoystick.GetRawButton(2))
  {
    IntakeLeft.Set(ControlMode::PercentOutput,-0.3);
    IntakeRight.Set(ControlMode::PercentOutput,0.3);
  }
  else
  {
    IntakeLeft.Set(ControlMode::PercentOutput,0);
    IntakeRight.Set(ControlMode::PercentOutput,0);
  }
  // Intake Arm Control
  if(DriverJoystick.GetRawButton(3))
  {
    IntakeArm.Set(ControlMode::PercentOutput,-0.3);
  }
  else if (DriverJoystick.GetRawButton(4))
  {
    IntakeArm.Set(ControlMode::PercentOutput,0.3);
  }
  else
  {
    IntakeArm.Set(ControlMode::PercentOutput,0);
  }
  //Drive Control
  if (DriverJoystick.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand)>0.5)
  {
  LeftDriveSpeed = 0.35*(DriverJoystick.GetX(frc::GenericHID::JoystickHand::kRightHand) - DriverJoystick.GetY(frc::GenericHID::JoystickHand::kLeftHand));
  RightDriveSpeed = 0.35 * (DriverJoystick.GetX(frc::GenericHID::JoystickHand::kRightHand) + DriverJoystick.GetY(frc::GenericHID::JoystickHand::kLeftHand));
  }
  else
  {
  LeftDriveSpeed = 0.7*(DriverJoystick.GetX(frc::GenericHID::JoystickHand::kRightHand) - DriverJoystick.GetY(frc::GenericHID::JoystickHand::kLeftHand));
  RightDriveSpeed = 0.7 * (DriverJoystick.GetX(frc::GenericHID::JoystickHand::kRightHand) + DriverJoystick.GetY(frc::GenericHID::JoystickHand::kLeftHand));
  }

  if(!DriverJoystick.GetBumper(frc::GenericHID::JoystickHand::kLeftHand))
  {
    if (LeftDriveSpeed > 1.0)
    {
      LeftDriveSpeed = 1.0;
    }
    else if (LeftDriveSpeed < -1.0)
    {
      LeftDriveSpeed = -1.0;
    }
    if (RightDriveSpeed > 1.0)
    {
      RightDriveSpeed = 1.0;
    }
    else if (RightDriveSpeed < -1.0)
    {
      RightDriveSpeed = -1.0;
    }
    if (DriverJoystick.GetY(frc::GenericHID::JoystickHand::kLeftHand)<Deadband && DriverJoystick.GetY(frc::GenericHID::JoystickHand::kLeftHand)>-Deadband && DriverJoystick.GetX(frc::GenericHID::JoystickHand::kRightHand)<Deadband && DriverJoystick.GetX(frc::GenericHID::JoystickHand::kRightHand)>-Deadband)
    {
    DriveLeft1.Set(ControlMode::PercentOutput, 0);
    DriveLeft2.Set(ControlMode::PercentOutput, 0);
    DriveRight1.Set(ControlMode::PercentOutput, 0);
    DriveRight2.Set(ControlMode::PercentOutput, 0);
    }
    else
    {
    DriveLeft1.Set(ControlMode::PercentOutput, LeftDriveSpeed);
    DriveLeft2.Set(ControlMode::PercentOutput, LeftDriveSpeed);
    DriveRight1.Set(ControlMode::PercentOutput, RightDriveSpeed);
    DriveRight2.Set(ControlMode::PercentOutput, RightDriveSpeed);
    }
  }
  // Solenoid Controls
  if (DriverJoystick.GetStickButton(frc::GenericHID::JoystickHand::kLeftHand))
  {
    ArmExtend.Set(true);
    ArmRetract.Set(false);
  }
  else if (DriverJoystick.GetStickButton(frc::GenericHID::JoystickHand::kRightHand))
  {
    ArmRetract.Set(true);
    ArmExtend.Set(false);
  }

  if (DriverJoystick.GetPOV()==0)
  {
    ShooterUp.Set(true);
    ShooterDown.Set(false);
    ShooterHigh = true;
  }
  else if(DriverJoystick.GetPOV()==180)
  {
    ShooterUp.Set(false);
    ShooterDown.Set(true);
    ShooterHigh = false;
  }

  if (SecondController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand)>0.5)
  {
    if (SecondController.GetBumper(frc::GenericHID::JoystickHand::kLeftHand))
    {
      ClimbLock.Set(true);
      ClimbRelease.Set(false);
    }
    else if(SecondController.GetBumper(frc::GenericHID::JoystickHand::kRightHand))
    {
      ClimbLock.Set(false);
      ClimbRelease.Set(true);
    }
  }
  //Lift Control

  if (SecondController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand)> 0.5)
  {
    double LiftSpeed = SecondController.GetY(frc::GenericHID::JoystickHand::kLeftHand);
    LiftMotor.Set(LiftSpeed);
  }
  else
  {
    LiftMotor.Set(0);
  }
}
void Robot::DisabledInit()
{
  
}
void Robot::DisabledPeriodic()
{

}
void Robot::TestInit()
{

}
void Robot::TestPeriodic()
{

}
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
