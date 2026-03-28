// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Agitator.Agitator;
import frc.robot.Shooter.ShooterConstants.HUBSTATE;
import frc.robot.Swerve.SwerveSubsystem;

public class AutoShoot extends Command {
  private final Feeder feeder;
  private final SwerveSubsystem swerve;
  private final Shooter shooter;
  private final Agitator agitator;
  private Translation3d HUB;
  boolean blue;

  public AutoShoot(Feeder Feeder, SwerveSubsystem Swerve, Shooter Shooter, Agitator Agitate) {
    swerve = Swerve;
    feeder = Feeder;
    shooter = Shooter;
    agitator = Agitate;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    blue = Shooter.getAlliance().get() == Alliance.Blue;

    if (blue) {
      HUB = ShooterConstants.TAGRETS.BLUEHUB;
    } else if (!blue) {
      HUB = ShooterConstants.TAGRETS.REDHUB;
    } else {
      HUB = ShooterConstants.TAGRETS.BLUEHUB;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ShooterCalculator.target(swerve.getPose(), blue) == HUB && ShooterCalculator.isHubActive() == HUBSTATE.ACTIVE) {
      feeder.shootOn();
      shooter.shooterGoShoot(true);
      agitator.spin();
    }
    else {
      feeder.shootOff();
      shooter.shooterGoShoot(false);
      agitator.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      feeder.shootOff();
      shooter.shooterGoShoot(false);
      agitator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
