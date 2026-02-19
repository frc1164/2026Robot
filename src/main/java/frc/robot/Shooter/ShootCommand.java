// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  private final Shooter ShooterSubsystem;

  private final double m_shootSpeed;
  private final double m_turnAngle;

  /** Creates a new ShootCommand. */
  public ShootCommand(double shootSpeed, double turnAngle, Shooter shooter) {
    ShooterSubsystem = shooter;

    m_shootSpeed = shootSpeed;
    m_turnAngle = turnAngle;

    addRequirements(ShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //subsystem.setPID(turnAngle*90)
    //ShooterSubsystem.setDriveVelocity(m_shootSpeed);
    ShooterSubsystem.runPID(m_turnAngle * 180);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("turnAngle", m_shootSpeed);
    return false;
  }
}
