// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  private final Shooter ShooterSubsystem;

  private final Supplier<Double> m_shootSpeed;
  private final Supplier<Double>  m_turnAngle;

  /** Creates a new ShootCommand. */
  public ShootCommand(Supplier<Double> shootSpeed, Supplier<Double> turnAngle, Shooter shooter) {
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
    ShooterSubsystem.setDriveVelocity(m_shootSpeed.get());
    SmartDashboard.putNumber("turnAngle", m_turnAngle.get());
    ShooterSubsystem.runPID(m_turnAngle.get() * 90);
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
