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

import frc.robot.Shooter.Feeder;
import frc.robot.Shooter.Shooter;



public class RobotContainer {
  private final Shooter shooter;
  private final Feeder feeder;

  private final CommandXboxController driveController;

  
  public RobotContainer() {
    feeder = new Feeder();

    driveController = new CommandXboxController(0);
        shooter = new Shooter(driveController);

    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
