// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Pickup extends Command {
  Intake intake;
  CommandXboxController controller;
  public Pickup(Intake m_intake, CommandXboxController m_controller) {
    intake = m_intake;
    controller = m_controller;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        intake.runPickup(controller.getRightY()/2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intake.runPickup(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
