/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/livewindow/LiveWindow.h>


Robot::Robot() {
  // Note SmartDashboard is not initialized here, wait until RobotInit() to make
  // SmartDashboard calls
  m_robotDrive.SetExpiration(0.1);
}

void Robot::RobotInit() {

 frc::SmartDashboard::PutNumber("LiftMotorPWR", 0.0);

  // Setup Camera server and set resolution and FPS
  camera0 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  camera0.SetResolution(320, 240);
  camera0.SetFPS(15);

#define ENABLE_CAMERA_2 1
#if ENABLE_CAMERA_2
  camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
  camera1.SetResolution(320,240);
  camera1.SetFPS(15);
#endif

// Enable(1)/Disable(0) Pneumatics in SW
//  Note: Define is in the header file.
#if CORJESU_TURNON_PNEUMATICS
   // Set Compressor Object for automatic closed loop control
   compressorObject.SetClosedLoopControl(true);
   // Set Solenoids to iniital stat
   //hatchDblSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
   hatchDblSolenoid.Set(frc::DoubleSolenoid::Value::kForward); 
   ballArmSolenoid.Set(false);
   climbBackDblSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); 
   climbFrontSolenoid.Set(false);
#endif

    // Initialize Arm Control Motors
    // NOTE:  This command does not chang the color of the TalonSRX output LEDs.  This may only be
    //        true for the TalonSRX
    ArmLiftMotor2.Set(ControlMode::Follower, 1);         // ArmLiftMotor 2 should follow 1
    ArmLiftMotor2.SetInverted(InvertType::OpposeMaster);  // Arm Motor 2 is inverted from Arm Motor 1
    ArmLiftMotor1.Set(ControlMode::PercentOutput, 0.0);  // Initialize to zero
    //ArmLiftMotor2.Set(ControlMode::PercentOutput, 0.0);

    ArmLiftMotor1.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    ArmLiftMotor2.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);



   // Initialize an encoder

    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 1 inch wheel with a 360 CPR encoder.
    //  Would be (Wheel Circumference * PI / 192)
    //  Now just convert to degree angle
    // Encoder on PRC Motor 180.0 pulses per rev ???
    // Encoder AMT102-V  Variable - now 2048 per rev
    //encArmCtrl.SetDistancePerPulse((360.0  / 2048.0));
    //encArmCtrl.SetDistancePerPulse((56.0  / 842.0));  // 56 deg by 842 counts
    encArmCtrl.SetDistancePerPulse((1.0));  // 56 deg by 842 counts
   
    
    encARMpid.SetSetpoint(0.0);
    encArmCtrl.Reset();  // Reset

  #if CORJESU_ENABLE_PID  
   // Enable PID Controller
// DISABLE FOR NOW:  encARMpid.Enable();
 #endif

#if 0
      // Create a 'DriveBase' tab and add the drivetrain object to it.
    frc::ShuffleboardTab& mR2JesuRobot = frc::Shuffleboard::GetTab("R2Jesu");
    mR2JesuRobot.Add("Drive", m_robotDrive);

    // Put encoders in a list layout.
 //   frc::ShuffleboardLayout& Myencoders =
 //       mR2JesuRobot.GetLayout("List Layout", "Encoders");

    mR2JesuRobot.Add("Arm Encoder", encArmCtrl);

    // Add a widget titled 'Max Speed' with a number slider.
    m_P_Gain = frc::Shuffleboard::GetTab("R2Jesu")
                     .Add("P Gain", 1)
                     .WithWidget("Number Slider")
                     .GetEntry();
#endif

}
 
/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
   // Currently no autonomous necessary.  We will use the camera system.
   // But we need to be able to run the system so use teleop control

    while (IsAutonomous() && IsEnabled()) 
      {
      R2JesuRobotControl();
      }
}

/**
 * Runs the motors with arcade steering.
 */
//void Robot::OperatorControl() 
void Robot::TeleopInit()
   {
   while (IsOperatorControl() && IsEnabled()) 
      {
      R2JesuRobotControl();
      }
   }

/**
 * Main function to control R2Jesu Robot.  
 * Called from both autonomous and telop modes
 */

void Robot::R2JesuRobotControl()
   {

    m_robotDrive.SetSafetyEnabled(true);

// ======================================================================================
// Robot Drive Train Control
// ======================================================================================
#if CORJESU_ENABLE_DRIVE
    // Speed of robot is controlled unless turbo botton is pressed.  B
    // But don't allow turbo button if arm is raised too high.
    if (m_stick.GetRawButton(1))                                // Trigger is brake
       m_robotDrive.ArcadeDrive(0.0, 0.0);
    else if (m_stick.GetRawButton(2) && !limitRobotSpeed)       // Turbo button
       m_robotDrive.ArcadeDrive(-m_stick.GetY(),m_stick.GetX());
    else                                                        // Half Speed
	   m_robotDrive.ArcadeDrive(-m_stick.GetY()*MaxRobotSpeed, m_stick.GetX()*MaxRobotSpeed);
#endif

//LED lights 
#if CORJESU_ENABLE_DRIVE
  if (m_stick.GetRawButton(3)) 
      blinkin.Set(ControlMode::PercentOutput, 0.73);
#endif

// ======================================================================================
// Robot Pnueumatic Control of Hatch Panel Release and Ball Cather Orientation
// ======================================================================================
#if CORJESU_TURNON_PNEUMATICS

   // Control the Ball Catcher Orientation Pneumatic
   // Up or Down
   if (m_OperatorJoyStick.GetZ() > 0.3)   // Make user depress axis switch some ammount
      {
      if (!ISBallCatherButtonPressed)  // Has operator requested an action
         {
          if (ISBallCatcherUp)         // What is the orientation of the ball cather
            {
            ISBallCatcherUp = false;
            hatchDblSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);   // Down
            }
         else  
            {
            ISBallCatcherUp = true;
            hatchDblSolenoid.Set(frc::DoubleSolenoid::Value::kForward);   // Up
            }
         }

//Back climb solenoid 
      /*   if (m_stick.GetRawButton(7)) {        
            if (ISBackClimbDown)
            {
            ISBackClimbDown = true;
            climbBackDblSolenoid.Set(frc::DoubleSolenoid::Value::kForward);   
            }
         else  
            {
            ISBackClimbDown = false;
            climbBackDblSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);   
            }
         } */
      // Set this flag after we change the ball catcher orenetation.  This flag
      // controls  that we don't change the oriention until the operator has released
      // the button   
      ISBallCatherButtonPressed = true;   
      }
   else
      ISBallCatherButtonPressed = false;  // Operator released button

   // Control the Hatch Panel Pneumatic Release
   // Action:  Release (Extended) or Retracted.  The logic should extend for some 
   // amount of time and then retract.  Currently the user hold the button to extend
   // the solenoid and it will retract when button is released.  Could use time.
   if (m_stick.GetRawButton(10) && ISBallCatcherUp)
      ballArmSolenoid.Set(true);       // Extend
   else
      ballArmSolenoid.Set(false);      // Retract

   if (m_stick.GetRawButton(8))
      climbFrontSolenoid.Set(true);      
   else
      climbFrontSolenoid.Set(false);      
   
   if (m_stick.GetRawButton(7))
      climbBackDblSolenoid.Set(frc::DoubleSolenoid::Value::kForward);   
   else
      climbBackDblSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); 
#endif

// ======================================================================================
// Robot Control of Arm Lift Motors
// ======================================================================================
    // Encoder Control
    // Control the Arm Motors
 
    // *******
    // Test Code
    // ******* 
  // Test Code to control arm motor
  const double SpeedDelta = 0.005;  //  Change motor speed in 5% increments
   if (m_OperatorJoyStick.GetRawButton(1))       // Emergency stop
      CurrentLiftMotorSpeed = 0.0;
   else if (m_OperatorJoyStick.GetRawButton(4))  // Increase by SpeedDelta
      CurrentLiftMotorSpeed = CurrentLiftMotorSpeed + SpeedDelta;
   else if (m_OperatorJoyStick.GetRawButton(2))  // Decrease by SpeedDelta
      CurrentLiftMotorSpeed = CurrentLiftMotorSpeed - SpeedDelta;
   else if (m_OperatorJoyStick.GetRawButton(3))  // Hold to use joystick
      CurrentLiftMotorSpeed = -m_OperatorJoyStick.GetY()/3.0;

   frc::SmartDashboard::PutNumber("LiftMotorPWR", CurrentLiftMotorSpeed);
 
   // Set motor to desired speed
   ArmLiftMotor1.Set(ControlMode::PercentOutput, CurrentLiftMotorSpeed);

#if CORJESU_ENABLE_PID  

//ArmLiftMotor2.Set(-ArmLiftMotor1.Get());
    
 PID_P = (frc::SmartDashboard::GetNumber("DB/Slider 0", 0.0) * 1000.0) + 
            (frc::SmartDashboard::GetNumber("DB/Slider 1", 0.0)  * 10000.0);
   frc::SmartDashboard::PutNumber("PID_P", PID_P);

   // Reset encodder, Disable or enable the PID
    if (m_stick.GetRawButton(3))   
       encArmCtrl.Reset();
    else if (m_stick.GetRawButton(4))  
       encARMpid.Disable();
    else if (m_stick.GetRawButton(6))  
        {
        encARMpid.SetPID(PID_P,0.0,0.0);
        encARMpid.Enable();
        encARMpid.SetSetpoint(100.0);
       }
    
    // Look at PID signals on the SmartDashboard
  // frc::SmartDashboard::PutNumber("Analog C1", analogJDCh1.GetValue());
  // frc::SmartDashboard::PutNumber("Analog C2", analogJDCh2.GetValue());
    frc::SmartDashboard::PutNumber("EncCount", encArmCtrl.Get());
    frc::SmartDashboard::PutNumber("EncCountRaw", encArmCtrl.GetRaw());
    frc::SmartDashboard::PutNumber("EncDist", encArmCtrl.GetDistance());
    frc::SmartDashboard::PutNumber("EncDistPerPuls", encArmCtrl.GetDistancePerPulse());
    frc::SmartDashboard::PutNumber("EncDir", encArmCtrl.GetDirection());
    frc::SmartDashboard::PutNumber("EncSamp", encArmCtrl.GetSamplesToAverage());
    frc::SmartDashboard::PutNumber("PIDSet", encARMpid.GetSetpoint());
    frc::SmartDashboard::PutNumber("PIDDeltaSet", encARMpid.GetDeltaSetpoint());
    frc::SmartDashboard::PutNumber("PIDerror", encARMpid.GetError());
  
    // Program different set points
    ArmScoring ArmScoringPos;

    if (ISBallCatcherUp)
       ArmScoringPos = R2Hatch;
    else
       ArmScoringPos = R2Cargo;
      
    if (m_OperatorJoyStick.GetRawButton(2))
        R2JesuArmSetPoint(ArmAngleArray[ArmScoringPos][R2low]);
    else  if (m_OperatorJoyStick.GetRawButton(3))
        R2JesuArmSetPoint(ArmAngleArray[ArmScoringPos][R2mid]);
    else  if (m_OperatorJoyStick.GetRawButton(4))
        R2JesuArmSetPoint(ArmAngleArray[ArmScoringPos][R2high]);
 
    // // Logic to adjust or limit robot speed depending on 
    // // height of the arm.  The flag is used to control turbo mode
    // if (encArmCtrl.GetDistance() >= ArmAngleArray[R2Hatch][R2mid])
    //    limitRobotSpeed = true;
    // else
    //    limitRobotSpeed = false;
    
#endif    

// ======================================================================================
// Robot Control of Arm Ball Spinner Motor
// ======================================================================================
  
  // Control Spinner Motor
   if (m_OperatorJoyStick.GetRawButton(6))  // Pick-up Ball
        m_SpinnerMotor.Set(0.65);
   else  if (m_OperatorJoyStick.GetRawButton(5))
        {
        // Logic for now    
        m_SpinnerMotor.Set(-1.0);  

        // The following code will control spinner motor speed based on arm angle.
        // TO DO:  Need to calibrate the angles and the motor speeds
        // if     (encArmCtrl.GetDistance() >= ArmAngleArray[R2Hatch][R2high])
        //    m_SpinnerMotor.Set(ballHigEjectMotorSpeed);
        // else if (encArmCtrl.GetDistance() >= ArmAngleArray[R2Hatch][R2mid])
        //    m_SpinnerMotor.Set(ballMidEjectMotorSpeed);
        // else 
        //    m_SpinnerMotor.Set(ballLowEjectMotorSpeed);

        }
   else  
       m_SpinnerMotor.Set(0.0);
  
    // The motors will be updated every 5ms
    frc::Wait(0.005);
  
}

// Function to control the arm discretes
void Robot::R2JesuArmSetPoint(double SetVal)
{
    double localSetValue = SetVal;
   
    // Check upper limit switch.  If true then reduce setpoint by 10%
    if (UpperArmLimitSwitch.Get())
        localSetValue = SetVal - 0.1*SetVal;

    // Check upper limit switch.  If true then increase setpoint by 10%
    if (lowerArmLimitSwitch.Get())
        localSetValue = SetVal + 0.1*SetVal;

      frc::SmartDashboard::PutNumber("localSetValue", localSetValue);
      frc::SmartDashboard::PutBoolean("Upper", UpperArmLimitSwitch.Get());
      frc::SmartDashboard::PutBoolean("Lower", lowerArmLimitSwitch.Get());

      encARMpid.SetSetpoint(localSetValue);

}

/**
 * Runs during test mode
 */
void Robot::TestInit() {
R2JesuRobotControl();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
