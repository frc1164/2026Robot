// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualShoot extends Command {
 private final Feeder feeder; 
 private final Shooter shooter;
 private final CommandXboxController controller;

 public ManualShoot(Feeder Feeder, CommandXboxController OperatorController, Shooter Shooter) {
   feeder = Feeder;
   controller = OperatorController;
   shooter = Shooter;
   addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.x().whileTrue(new ParallelCommandGroup(new InstantCommand(() -> feeder.shootHeld(.5)), new InstantCommand(() -> shooter.setShotSpeed(4000))));
    controller.x().whileFalse(new ParallelCommandGroup(new InstantCommand(() -> feeder.shootHeld(0)), new InstantCommand(() -> shooter.setShotSpeed(0))));


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
