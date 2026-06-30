// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Retract extends Command {
  /** Creates a new Retract. */
  Intake intake;
  Shooter shooter;
  public Retract(Shooter m_shooter, Intake m_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = m_shooter;
    intake = m_intake;
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.retract();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runThetaPID(90);
    shooter.runPhiPID(95);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // CommandScheduler.getInstance().schedule(new Extend(intake, shooter));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
