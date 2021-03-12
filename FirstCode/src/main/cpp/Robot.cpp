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
  frc::SmartDashboard::PutNumber("Program",0);
  Selected = 0;
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
  if(Selected == 0)
  {
    int num = frc::SmartDashboard::GetNumber("Program",0);
    if(num == 1)
    {
      Selected = 1;
      GalacticSearch();
    }
    else if(num == 2)
    {
      Selected = 1;
      AutoNav();
    }
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
      SetDrive(adjust,adjust);
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
      SetShooters(0.35);
    }
    else if(DriverJoystick.GetBumper(frc::GenericHID::JoystickHand::kRightHand))
    {
      SetShooters(0.55);
    }
    else
    {
      SetShooters(0.45);
    }
  }
  else
  {
    SetShooters(0);
  }
  //Conveyor Control
  if(DriverJoystick.GetRawButton(1))
  {
    SetIntake(0.3);
  }
  else if(DriverJoystick.GetRawButton(2))
  {
    SetIntake(-0.3);
  }
  else
  {
    SetIntake(0);
  }
  // Intake Arm Control
  if(DriverJoystick.GetRawButton(3))
  {
    IntakeArm.Set(-0.3);
  }
  else if (DriverJoystick.GetRawButton(4))
  {
    IntakeArm.Set(0.3);
  }
  else
  {
    IntakeArm.Set(0);
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
      SetDrive(0,0);
    }
    else
    {
      SetDrive(LeftDriveSpeed, RightDriveSpeed);
    }
  }
  // Solenoid Controls
  if (DriverJoystick.GetStickButton(frc::GenericHID::JoystickHand::kLeftHand))
  {
    SwitchThings(0,1);
  }
  else if (DriverJoystick.GetStickButton(frc::GenericHID::JoystickHand::kRightHand))
  {
    SwitchThings(0,0);
  }

  if (DriverJoystick.GetPOV()==0)
  {
    SwitchThings(1,1);
  }
  else if(DriverJoystick.GetPOV()==180)
  {
    SwitchThings(1,0);
  }

  if (SecondController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand)>0.5)
  {
    if (SecondController.GetBumper(frc::GenericHID::JoystickHand::kLeftHand))
    {
      SwitchThings(2,1);
    }
    else if(SecondController.GetBumper(frc::GenericHID::JoystickHand::kRightHand))
    {
      SwitchThings(2,0);
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
void Robot::SetShooters(float n)
{
  ShooterTop.Set(ControlMode::PercentOutput,n);
  ShooterBottom.Set(ControlMode::PercentOutput,n);
}
void Robot::SetDrive(float n,float x)
{
  DriveLeft1.Set(ControlMode::PercentOutput, n);
  DriveLeft2.Set(ControlMode::PercentOutput, n);
  DriveRight1.Set(ControlMode::PercentOutput, x);
  DriveRight2.Set(ControlMode::PercentOutput, x);
}
void Robot::SetIntake(float n)
{
  IntakeLeft.Set(n);
  IntakeRight.Set(-1*n);
}
void Robot::SwitchThings(int n,bool t)
{
  switch(n)
  {
    case 0:
      ArmExtend.Set(t);
      ArmRetract.Set(!t);
      break;
    case 1:
      ShooterUp.Set(t);
      ShooterDown.Set(!t);
      ShooterHigh = t;
      break;
    case 2:
      ClimbLock.Set(t);
      ClimbRelease.Set(!t);
      break;
    default:
      //should never go through default. If it did, wrong number was passed.
      break;
  }
}
void Robot::VisionThread()
{
  //Copied from wpilib. TODO: get it working and change it to work with network tables or something to scan for ball?
  // Creates UsbCamera and MjpegServer [1] and connects them
  frc::CameraServer::GetInstance()->StartAutomaticCapture();
  // Creates the CvSink and cmonnects it to the UsbCamera
  cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
  //eates the CvSource and MjpegServer [2] and connects them
  cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Blur", 640, 480);
  std::cout << "Jerry was here?!?!\n";
}
void Robot::GalacticSearch()
{
  //SetDrive(0.2,0.2);
  //std::thread visionThread(VisionThread);
  //visionThread.detach();
  VisionThread();
}
void Robot::AutoNav()
{
  SwitchThings(0,1);
  SwitchThings(0,0);
}
/*frc::Command* Robot::GetAutonomousCommand()
{
  return m_chooser.GetSelected();
}*/
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
