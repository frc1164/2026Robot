// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Agitator.Agitator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UnJamTheShooterLikeABoss extends Command {
  /** Creates a new UnJamTheShooterLikeABoss. */
  Shooter shooter;
  Agitator agitator;
  Feeder feeder;
  public UnJamTheShooterLikeABoss(Shooter Shooter, Agitator Agitator, Feeder Feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter;
    agitator = Agitator;
    feeder = Feeder;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.shooterGoShoot(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.reverse();
    agitator.reverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shooterGoShoot(false);
    agitator.stop();
    feeder.shootOff();
    shooter.resetLastSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
