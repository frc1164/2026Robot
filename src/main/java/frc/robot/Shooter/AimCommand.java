// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterCalculator.ShotInfo;
import frc.robot.Swerve.SwerveSubsystem;

public class AimCommand extends Command {
  private final Shooter ShooterSubsystem;


  private final SwerveSubsystem Swerve;
  public AimCommand(Shooter shooter, SwerveSubsystem swerve) {
    ShooterSubsystem = shooter;
    Swerve = swerve;

    addRequirements(ShooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //Get robot/shooter current location
    Pose2d botPose = Swerve.getPose();

    //Get initial target
    Translation3d target = ShooterCalculator.target(Swerve.getPose());

    //Calculate optimal shot/aiming
    ShotInfo shot = ShooterCalculator.getShot(Swerve.fieldRelativeVelocity(), target, botPose, 4);

    //Feed it into the shooter
    double theta = ShooterCalculator.getThetaAngle(ShooterCalculator.distVector(new Pose2d(shot.getTarget().getX(), shot.getTarget().getY(), null), botPose), botPose);
    ShooterSubsystem.runThetaPID(theta);
    ShooterSubsystem.runPhiPID(shot.getVertAngle());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
