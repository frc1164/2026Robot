// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Swerve.SwerveSubsystem;

public class ShooterCalculator extends SubsystemBase {
  /** Creates a new ShooterCalculator. */
  SwerveSubsystem swerve;
  double offsetMagnitude = 0; //ShooterConstants
  double offsetVert = 0;
  double offsetAngleRad = 0; //ShooterConstants
  double shooterSpeed = 0; //also ShooterConstants
  double targetH = 0; //also ShooterConstants
  double G = 9.81; //also ShooterConstants but might need to be negative not sure yet


  public ShooterCalculator() {}

  private Translation2d distVector(Pose2d target, Pose2d current){//target pose in constants
    double xDist, yDist;

    xDist = target.getX() + offsetMagnitude * (Math.cos(offsetAngleRad + current.getRotation().getRadians()));
    yDist = target.getY() + offsetMagnitude * (Math.sin(offsetAngleRad + current.getRotation().getRadians()));
    return new Translation2d(xDist, yDist);
  }

  private double getDist(Translation2d distVector){
    return Math.sqrt(Math.pow(distVector.getX(), 2) + Math.pow(distVector.getY(), 2));
  }

  public double phiAnglefromVelo(Pose2d target, Pose2d shooterLocation){//target pose in constants
    double angle;
    double h = (targetH - offsetVert);
    angle = Math.atan((shooterSpeed + Math.sqrt(Math.pow(shooterSpeed,4)- (G) * (G * (getDist(distVector(target, shooterLocation)) * getDist(distVector(target, shooterLocation)) + 2 * (shooterSpeed * shooterSpeed) * h)))
                        / (G * getDist(distVector(target, shooterLocation)))));
    return angle;
  }

  public double calcT(double vertAngle, double dist){
    return dist/(shooterSpeed * Math.cos(vertAngle));
  }

  public double botRelativeThetaNoVelRad(Translation2d distanceVector){
    return distanceVector.getAngle().getRadians(); 
  }


  public double botThetaWithVelRad(double time, Translation2d botVelo, Translation2d distance){
    return Math.atan2(distance.getY() + (botVelo.getY() * time), distance.getX() + (botVelo.getX() * time));
  }

  public record ShotInfo(double exitVel, double vertAngle, Translation3d target){
    public double getZComponent(){
      return exitVel * Math.sin(vertAngle);
    }

    public double getXYComponent(){
      return exitVel * Math.cos(vertAngle);
    }

    public double getMag(){
      return exitVel;
    }

    public double getVertAngle(){
      return vertAngle();
    }

    public Translation3d getTarget(){
      return target();
    }

    public static ShotInfo interpolate(ShotInfo estimate, ShotInfo result, double t){
      return new ShotInfo(estimate.getMag(),
                  MathUtil.interpolate(estimate.getVertAngle(), result.getVertAngle(), t),
                  result.getTarget());
    }
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
