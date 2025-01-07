// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.swerve.Drive;
import frc.robot.swerve.Drive.DriveState;
import frc.robot.swerve.Gyro.GyroHardware;
import frc.robot.swerve.SwerveModule.SwerveModule;
import frc.robot.swerve.SwerveModule.SwerveModuleHardware;
import static frc.robot.swerve.DriveConstants.*;

public class RobotContainer {
  public CommandXboxController driveController = new CommandXboxController(0);
  public Drive drive;
  public SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drive = new Drive(new SwerveModule[] {
      new SwerveModule(new SwerveModuleHardware(frontLeft)),
      new SwerveModule(new SwerveModuleHardware(frontRight)),
      new SwerveModule(new SwerveModuleHardware(backLeft)),
      new SwerveModule(new SwerveModuleHardware(backRight))
  }, new GyroHardware());

    autoChooser = AutoBuilder.buildAutoChooser();  

    drive.acceptJoystickInputs(
      () -> driveController.getLeftY(), 
      () -> driveController.getLeftX(), 
      () -> driveController.getRightX());

    drive.setDefaultCommand(Commands.run(() -> drive.setDriveEnum(DriveState.TELEOP), drive));
    
    configureTriggers();
    configureBindings();
  }

  private void configureTriggers() {
    new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> drive.resetAllEncoders()));
  }
  
  private void configureBindings() {
    driveController.x().onTrue(Commands.runOnce(() -> {drive.resetGyro();}));

    driveController.povUp().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_UP)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));
        
    driveController.povRight().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_RIGHT)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

    driveController.povDown().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_DOWN)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

    driveController.povLeft().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_LEFT)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

    driveController.a().onTrue(drive.characterizeDriveMotors()).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));
  }

  public Command getAutonomousCommand() {
    return drive.setDriveStateCommand(DriveState.AUTON).andThen(autoChooser.getSelected());
  }
}
