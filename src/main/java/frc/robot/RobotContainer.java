// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Shooter.AimCommand;
import frc.robot.Shooter.AutoShoot;
import frc.robot.Shooter.Feeder;
import frc.robot.Shooter.ManualShoot;
import frc.robot.Shooter.Shooter;
import frc.robot.Swerve.SwerveJoystickCmd;
import frc.robot.Swerve.SwerveSubsystem;


public class RobotContainer {
  private final SwerveSubsystem swerve;
  private final Shooter shooter;
  private final Feeder feeder;

  private final CommandXboxController driveController, operatorController;

  private final SendableChooser<Command> autoChooser;
  
  @SuppressWarnings("unused")
  private final LEDs leds = new LEDs();

  public RobotContainer() {
    swerve = new SwerveSubsystem();
    feeder = new Feeder();
    shooter = new Shooter(feeder);

    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    swerve.setDefaultCommand(new SwerveJoystickCmd(
      swerve,
      () -> driveController.getLeftY(),
      () -> driveController.getLeftX(),
      () -> -driveController.getRightX(),
      () -> !driveController.povUp().getAsBoolean()));
    
    shooter.setDefaultCommand(new AimCommand(shooter, swerve));

    //this SHOULD be overwritten by auton during auton period I hope, if not then this gets problematic
    feeder.setDefaultCommand(new AutoShoot(feeder, swerve));
    

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    driveController.povDown().onTrue(new InstantCommand(() -> swerve.zeroHeading()));
    operatorController.povDown().toggleOnTrue(new ManualShoot(feeder, operatorController));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
