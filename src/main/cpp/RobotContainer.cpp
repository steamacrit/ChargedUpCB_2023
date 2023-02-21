// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>


#include "commands/Autos.h"
#include "commands/ExampleCommand.h"


std::shared_ptr<RobotContainer> g_container{ nullptr };
std::shared_ptr<RobotContainer> RobotContainer::Get() {
    if (!g_container)
        g_container.reset(new RobotContainer());

    return g_container;
}

RobotContainer::RobotContainer() 
    : PneumaticsRunning(false)
    , WristDegrees(0.0)
    , ArmDegrees(0.0)
    , CorrectionValue(0.0) {

    DriverControl.reset(new t34::T34XboxController(ID_DRIVE_CONTROLLER)); 
    DriverControl->setAllAxisDeadband(0.07);

    Drive.reset(new t34::SwerveDrive());

    ArmMotor.reset(new TalonSRX(ID_ARM_MOTOR));
    ArmExtensionMotor.reset(new rev::CANSparkMax(ID_ARM_EXT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
    ArmEncoder.reset(new frc::Encoder(0,1));
    
    WristYMotor.reset(new TalonSRX(ID_WRIST_Y_MOTOR));
    WristRotateMotor.reset(new TalonSRX(ID_WRIST_ROT_MOTOR));
    WristEncoder.reset(new frc::Encoder(2,3));
    WristPID.reset(new frc2::PIDController(0.05, 0.0, 0.0));

    GripSolenoid.reset(new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, 0));
    Compressor.reset(new frc::Compressor(1, frc::PneumaticsModuleType::CTREPCM));

    // Initialize all of your commands and subsystems here
        DefaultCommand.reset(new t34::DefaultDriveCommand(Drive, DriverControl)); 

    // Configure the button bindings
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    // Configure your trigger bindings here

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //frc2::Trigger([this] { return m_subsystem.ExampleCondition(); }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

    // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    //m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous
    return DefaultCommand.get();
}
