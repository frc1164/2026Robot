// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterCalculator extends SubsystemBase {

  public ShooterCalculator() {}

  public static Translation2d distVector(Pose2d target, Pose2d current){//target pose in constants
    double xDist, yDist;
    double botDiffX = target.getX() - current.getX();
    double botDiffY = target.getY() - current.getY();
    SmartDashboard.putNumber("currentX", current.getX());
    xDist = botDiffX + ShooterConstants.SHOOTEROFFSETS.translation * (Math.cos(ShooterConstants.SHOOTEROFFSETS.theta + current.getRotation().getRadians()));
    SmartDashboard.putNumber("xDist", xDist);
    yDist = botDiffY + ShooterConstants.SHOOTEROFFSETS.translation * (Math.sin(ShooterConstants.SHOOTEROFFSETS.theta + current.getRotation().getRadians()));
    SmartDashboard.putNumber("yDist", yDist);
    return new Translation2d(xDist, yDist);
  }

  private static double getDist(Translation2d distVector){
    return Math.sqrt(Math.pow(distVector.getX(), 2) + Math.pow(distVector.getY(), 2));
  }

  public static double phiAnglefromVelo(Pose2d target, Pose2d shooterLocation){//target pose in constants
    double angle;
    double h = (ShooterConstants.hubPose.getZ() - ShooterConstants.SHOOTEROFFSETS.vertical);
    angle = Math.atan((ShooterConstants.exitVelocity + Math.sqrt(Math.pow(ShooterConstants.exitVelocity,4)- (ShooterConstants.gravity) * (ShooterConstants.gravity * (getDist(distVector(target, shooterLocation)) * getDist(distVector(target, shooterLocation)) + 2 * (ShooterConstants.exitVelocity * ShooterConstants.exitVelocity) * h)))
                        / (ShooterConstants.gravity * getDist(distVector(target, shooterLocation)))));
    return angle;
  }

  public static double calcT(double vertAngle, double dist){
    return dist/(ShooterConstants.exitVelocity * Math.cos(vertAngle));
  }

  public static double botRelativeThetaNoVelRad(Translation2d distanceVector){
    return distanceVector.getAngle().getRadians(); 
  }


  public static double botThetaWithVelRad(double time, Translation2d botVelo, Translation2d distance){
    return Math.atan2(distance.getY() + (botVelo.getY() * time), distance.getX() + (botVelo.getX() * time));
  }


  public Translation3d predictTargetpose(Translation3d target, double time, ChassisSpeeds velocity){
    double xEstimate = target.getX() - velocity.vxMetersPerSecond * time;
    double yEstimate = target.getY() - velocity.vyMetersPerSecond * time;
    return new Translation3d(xEstimate, yEstimate, target.getZ());
  }

  public ShotInfo iterateEstimatedShotInfo(ChassisSpeeds velocity, Translation3d target, Pose2d botPose, int iterations){
    double dist = botPose.getTranslation().getDistance(target.toTranslation2d());
    ShotInfo SHOT = ShooterConstants.shotMap.get(dist);
    SHOT = new ShotInfo(SHOT.exitVel(), SHOT.getVertAngle(), target);
    double time = ShooterConstants.timeMap.get(dist);
    Translation3d predictedTarget = target;

    for (int i = 0; i < iterations; i++){
      predictedTarget = predictTargetpose(target, time, velocity);
      dist = botPose.getTranslation().getDistance(predictedTarget.toTranslation2d());
      SHOT = ShooterConstants.shotMap.get(dist);
      SHOT = new ShotInfo(SHOT.exitVel(), SHOT.getVertAngle(), predictedTarget);
      time = ShooterConstants.timeMap.get(dist);
    }
    return SHOT;
  }


  public record ShotInfo(double exitVel, double vertAngle, Translation3d target){
    public ShotInfo(double exitVel, double vertAngle){
      this(exitVel, vertAngle, ShooterConstants.hubPose);
    }
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
