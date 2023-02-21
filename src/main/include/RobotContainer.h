// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/CommandXboxController.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <ctre/Phoenix.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <AHRS.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>

#include "Constants.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/ExampleSubsystem.h"
#include "utils/T34XboxController.h"
#include "commands/CMD_DefaultDrive.h"
#include "subsystems/ClawSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:

    static std::shared_ptr<RobotContainer> Get();

    std::shared_ptr<t34::T34XboxController> DriverControl;

    std::shared_ptr<t34::SwerveDrive> Drive; 

// This is state that should be part of the SwerveDrive class
//    bool DriveBraking;

    std::shared_ptr<TalonSRX> ArmMotor;
    std::shared_ptr<rev::CANSparkMax> ArmExtensionMotor;
    std::shared_ptr<frc::Encoder> ArmEncoder;
    double ArmDegrees;

    std::shared_ptr<TalonSRX> WristYMotor;
    std::shared_ptr<TalonSRX> WristRotateMotor;
    std::shared_ptr<frc::Encoder> WristEncoder;
    std::shared_ptr<frc2::PIDController> WristPID;
    double WristDegrees;

    std::shared_ptr<frc::Solenoid> GripSolenoid;
    bool PneumaticsRunning;
    std::shared_ptr<frc::Compressor> Compressor;

    double CorrectionValue;

    //  COMMANDS
    std::shared_ptr<t34::DefaultDriveCommand> DefaultCommand;

    frc2::CommandPtr GetAutonomousCommand();

private:
    RobotContainer();
    void ConfigureBindings();
};
