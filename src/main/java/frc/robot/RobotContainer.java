// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Shooter.ShootCommand;
import frc.robot.Shooter.Shooter;
import frc.robot.Swerve.SwerveJoystickCmd;
import frc.robot.Swerve.SwerveSubsystem;


public class RobotContainer {
  private final SwerveSubsystem swerve;
  private final Shooter shooter;

  private final CommandXboxController driveController;

  public RobotContainer() {
    swerve = new SwerveSubsystem();
    shooter = new Shooter();

    driveController = new CommandXboxController(0);

    swerve.setDefaultCommand(new SwerveJoystickCmd(
      swerve,
      () -> driveController.getLeftY(),
      () -> driveController.getLeftX(),
      () -> -driveController.getRightX(),
      () -> !driveController.povUp().getAsBoolean()));

    shooter.setDefaultCommand(new ShootCommand(
      () -> driveController.getLeftTriggerAxis(),
      () -> driveController.getRightTriggerAxis(),
       shooter));
    configureBindings();
  }

  private void configureBindings() {
    driveController.povDown().onTrue(new InstantCommand(() -> swerve.zeroHeading()));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
