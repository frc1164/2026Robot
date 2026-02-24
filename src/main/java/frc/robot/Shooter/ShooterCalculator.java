// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Swerve.SwerveSubsystem;

public class ShooterCalculator extends SubsystemBase {
  /** Creates a new ShooterCalculator. */
  Pose2d pose;
  Pose3d target;
  SwerveSubsystem swerve;
  double offsetMagnitude = 0; //ShooterConstants
  double offsetVert = 0;
  double offsetAngleRad = 0; //ShooterConstants
  double shooterSpeed = 0; //also ShooterConstants
  double G = 9.81; //also ShooterConstants but might need to be negative not sure yet


  public ShooterCalculator(SwerveSubsystem chassis) {
    swerve = chassis;
    pose = swerve.getPose();
    target = new Pose3d(); //Find coords for target and get the 'optimal' height also create a ShooterConstants for all these
  }

  private double getDist(){
    double xDist, yDist, totalDist;

    xDist = target.getX() + offsetMagnitude * (Math.cos(offsetAngleRad + pose.getRotation().getRadians()));
    yDist = target.getY() + offsetMagnitude * (Math.sin(offsetAngleRad + pose.getRotation().getRadians()));
    totalDist = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
    return totalDist;
  }

  public double phiAnglefromVelo(){
    double angle;
    double h = (target.getZ() - offsetVert);
    angle = Math.atan((shooterSpeed + Math.sqrt(Math.pow(shooterSpeed,4)- (G) * (G * (getDist() * getDist()) + 2 * (shooterSpeed * shooterSpeed) * h)))
                        / (G * getDist()));
    return 0;
  }

  public double tFromPhi(){
    return 0;
  }

  public double thetaAngle(){
    return 0;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
