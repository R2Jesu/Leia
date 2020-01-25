/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

// Shuffleboard includes
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

// WPI LIB Includes
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SendableChooser.h>

// Include TalonSRX Libraries
#include <ctre/Phoenix.h>

// Header file for camera server
#include <cameraserver/CameraServer.h>

// Header file for controlling a compressor and solenoid
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>

// Header file for encoders
#include <frc/Encoder.h>
#include <frc/AnalogInput.h>

// PID Controller
#include <frc/PIDController.h>

// Digital Input for limit switches
#include <frc/DigitalInput.h>

// R2Jesu Code Control Sections to turn subsystems on and off

// Enable(1)/Disable(0) Pneumatic in SW
#define CORJESU_TURNON_PNEUMATICS 1

// Enable(1)/Disable(0) Pneumatic in SW
#define CORJESU_ENABLE_PID 0

// Enable(1)/Disable(0) Pneumatic in SW
#define CORJESU_ENABLE_DRIVE 1

/**
 * This is a demo program showing the use of the DifferentialDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use TimedRobot or Command-Based
 * instead if you're new.
 */
class Robot : public frc::TimedRobot {
 public:
  Robot();

  void RobotInit() override;
  void AutonomousInit() override;
  void TeleopInit() override;
  //void OperatorControl() override;
  void TestInit() override;

 private:

  nt::NetworkTableEntry m_P_Gain;

  // Robot drive system
  frc::PWMVictorSPX m_leftMotor{0};
  frc::PWMVictorSPX m_rightMotor{1};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
 
 //LEDs 
  WPI_TalonSRX blinkin{3};

  // Robot Accessory Control
  WPI_TalonSRX  ArmLiftMotor1{1};  // These 2 motors are slaved to drive main lift arm
  WPI_TalonSRX  ArmLiftMotor2{2};
  frc::PWMVictorSPX m_SpinnerMotor{2};  // Drives spinner

  // Drive Joystick
  frc::Joystick m_stick{0};
  
  //Operator Joystick
  frc::Joystick m_OperatorJoyStick{1};

  // Compressor and Solenoid Control
  frc::Compressor compressorObject;
  frc::DoubleSolenoid hatchDblSolenoid{0,1};  // Hatch Control
  frc::Solenoid ballArmSolenoid{2};           // Ball Catcher Control
  frc::DoubleSolenoid climbBackDblSolenoid{5,4};  
  frc::Solenoid climbFrontSolenoid{3};           
                                                   
  // Camera Objects
  cs::UsbCamera camera0;
  cs::UsbCamera camera1;

  // Encoder objects
  frc::Encoder encArmCtrl{0,1,true,frc::Encoder::EncodingType::k4X};

  // PID Controller 
  double PID_P = 1000.0;  // Initial Gain
  frc::PIDController encARMpid{PID_P,0.0,0.0,&encArmCtrl,&ArmLiftMotor1};

  // Robot Control Objects
  frc::DigitalInput  lowerArmLimitSwitch {2};
  frc::DigitalInput  UpperArmLimitSwitch {3};

  // Control the speed of the robot if the arm is above a certain angle.
  bool   limitRobotSpeed = false;
  const double MaxRobotSpeed = 0.65;  // Percentage limit

  // Control the speed of the spinner motor ball ejection based on arm angle
  // TO DO: Tune these speeds
  const double ballIntakeMotorSpeed   = 0.5;
  const double ballLowEjectMotorSpeed = 0.6;
  const double ballMidEjectMotorSpeed = 0.7;
  const double ballHigEjectMotorSpeed = 0.9;

  // Define the arm angles in degrees
  // TO DO: Tune these angles
  enum ArmScoring { R2Hatch, R2Cargo};
  enum ArmAngle { R2low, R2mid, R2high };

  const double ArmAngleArray[2][3] = {{10.0, 30.0, 50.0},
                                      {15.0, 35.0, 55.0}};
  
  bool ISBallCatcherUp            = true;
  bool ISBallCatherButtonPressed  = false;

  bool ISBackClimbDown = false;



  // Logic to control Lift Motor 
  double CurrentLiftMotorSpeed = 0.0;

  // The John Deere Rotary Encoder outputs an analog voltage signal.
  // frc::AnalogInput analogJDCh1{0};
  // frc::AnalogInput analogJDCh2{1};

  // Subfuntion to control the robot.  This function will be called from both 
  // autonomous and telop 
  void R2JesuArmSetPoint(double SetVal = 0.0);
  void R2JesuRobotControl();
  
};