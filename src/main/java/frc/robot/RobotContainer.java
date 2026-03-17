// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Shooter.Shooter;


public class RobotContainer {
  private final Shooter shooter;

  private final CommandXboxController driveController;
  public RobotContainer() {
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
