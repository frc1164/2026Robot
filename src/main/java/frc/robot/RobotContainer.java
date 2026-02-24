// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Climber.Climb;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Swerve.SwerveJoystickCmd;
import frc.robot.Swerve.SwerveSubsystem;


public class RobotContainer {
  private Climb m_climb = new Climb();
  private final SwerveSubsystem swerve;
  private final CommandXboxController driveController;

  public RobotContainer() {
    swerve = new SwerveSubsystem();
    driveController = new CommandXboxController(0);

    swerve.setDefaultCommand(new SwerveJoystickCmd(
      swerve,
      () -> driveController.getLeftY(),
      () -> driveController.getLeftX(),
      () -> -driveController.getRightX(),
      () -> !driveController.povUp().getAsBoolean()));

    configureBindings();
  }

  private void configureBindings() {
    driveController.povDown().onTrue(new InstantCommand(() -> swerve.zeroHeading()));
    driveController.x().onTrue(new InstantCommand(()-> m_climb.extend()));
    driveController.b().onTrue(new InstantCommand(()-> m_climb.retract()));
  }
 
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
