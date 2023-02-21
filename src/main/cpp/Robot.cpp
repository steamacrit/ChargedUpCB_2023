// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>


Robot::Robot() {}

void Robot::RobotInit() {
    m_container = RobotContainer::Get();

    frc2::CommandScheduler::GetInstance().SetDefaultCommand(m_container->Drive.get(), *(m_container->DefaultCommand.get()));

    m_container->ArmMotor->SetNeutralMode(NeutralMode::Brake);
    m_container->ArmEncoder->Reset();

    m_container->WristYMotor->SetNeutralMode(NeutralMode::Brake);
    m_container->WristRotateMotor->SetNeutralMode(NeutralMode::Brake);
    m_container->WristEncoder->Reset();
    m_container->WristEncoder->SetDistancePerPulse(360.0 / 44.4);
    m_container->WristPID->SetTolerance(2, 4);

    m_container->GripSolenoid->Set(0);

// ??? WHY DISABLE ???
    m_container->Compressor->Disable();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
    //  Converting the encoder values of the arm motor and wrist y encoder to degrees ^
    m_container->WristDegrees    = t34::EncoderToDegree(44.4, m_container->WristEncoder->Get());
    m_container->ArmDegrees      = t34::EncoderToDegree(44.4, m_container->ArmEncoder->Get());
    m_container->CorrectionValue = t34::CorrectionValue(m_container->ArmDegrees, m_container->WristDegrees) * -1.0;

    //  Setting the speed of the wrist_Y motor based on PID ^
    m_container->WristYMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_container->WristPID->Calculate(m_container->CorrectionValue * -0.2);

// REMOVE IF NOT NEEDED DURING COMPETITION
    frc::SmartDashboard::PutNumber("W Distance per pulse", RobotContainer::wrist_encoder->GetDistancePerPulse());
    frc::SmartDashboard::PutNumber("W Current Position", RobotContainer::wrist_degrees);
    frc::SmartDashboard::PutNumber("A Distance per pulse", RobotContainer::arm_encoder->GetDistancePerPulse());
    frc::SmartDashboard::PutNumber("A Current Position", RobotContainer::arm_degrees);
    frc::SmartDashboard::PutNumber("W Motor Current", RobotContainer::m_wrist_y->GetOutputCurrent());
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    m_container->Drive->zeroYaw();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
    frc::SmartDashboard::PutBoolean("Pneumatics Running", m_container->PneumaticsRunning);
    frc::SmartDashboard::PutBoolean("Drive Brake On", m_container->DriveBraking);

    double right_Y = m_container->DriverControl->getRightStickYDB();
    //  Arm Pitch Control
// DID THIS EVEN COMPILE? 
//    if (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, RobotContainer::m_driver_control->getRightStickYDB() >= 0.5 || RobotContainer::m_driver_control->getRightStickYDB() <= 0.5)
    if (right_Y >= 0.5 || right_Y <= 0.5)
        m_container->ArmMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, right_Y * 1.0);

    //  Wrist Rotation Control
    if (m_container->DriverControl->GetRightBumperPressed())
        m_container->WristRotateMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.2);

    if (m_container->DriverControl->GetLeftBumperPressed())
        m_container->WristRotateMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.2);

    //  Arm Extension Control
    int pov = m_container->DriverControl->GetPOV();

    if (pov == 0)
        m_container->ArmExtentionMotor->Set(0.2);
    else if (pov == 180)
        m_container->ArmExtensionMotor->Set(-0.2);
    else 
        m_container->ArmExtention->Set(0.0);

// ??? WHY ???
// We never manually start or stop the compressor. The compressor should begine to run once the robot is enabled
// and continue to run until the robot is disabled. The pressure switch turns the compressor on or off as needed.
    //  Compressor Toggle
    /*
    if (RobotContainer::m_driver_control->GetStartButtonReleased())
    RobotContainer::pneumatics_running = ! RobotContainer::pneumatics_running;

    if (RobotContainer::pneumatics_running)
    RobotContainer::p_grip_compressor->EnableDigital();

    else if(RobotContainer::pneumatics_running == false)
    RobotContainer::p_grip_compressor->Disable();
    */

    //  Grip Solenoid
    if (m_container->DriverControl->GetXButtonReleased())
        m_container->GripSolenoid->Toggle();

    //  SheildWall & Toggle Drive Break
    if (RobotContainer::m_driver_control->GetBackButtonReleased()) {
        m_container->Drive->sheildWall();

        // NO
        // Drive braking should not be toggled. It should be set to on.
        // Once we start driving again it should set to our preferred 
        // state. See me for solutions.
        /*
        RobotContainer::m_drive->toggleDriveBrake();
        RobotContainer::drive_braking = !RobotContainer::drive_braking;
        */
    }

}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif
