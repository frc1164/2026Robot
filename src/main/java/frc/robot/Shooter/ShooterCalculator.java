// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Shooter.ShooterConstants.HUBSTATE;

public class ShooterCalculator {
  public ShooterCalculator() {
  }

  //Flat ground dist from shooter to target
  public static Translation2d distVector(Pose2d target, Pose2d current) {
    double xDist, yDist;
    double botDiffX = target.getX() - current.getX();
    double botDiffY = target.getY() - current.getY();
    SmartDashboard.putNumber("currentX", current.getX());
    xDist = botDiffX - ShooterConstants.SHOOTEROFFSETS.translation
        * (Math.cos(ShooterConstants.SHOOTEROFFSETS.theta + current.getRotation().getRadians()));
    SmartDashboard.putNumber("xDist", xDist);
    yDist = botDiffY - ShooterConstants.SHOOTEROFFSETS.translation
        * (Math.sin(ShooterConstants.SHOOTEROFFSETS.theta + current.getRotation().getRadians()));
    SmartDashboard.putNumber("yDist", yDist);
    return new Translation2d(xDist, yDist);
  }

  // Just pythagoran applied to whatever given vector
  private static double getDist(Translation2d distVector) {
    return Math.sqrt(Math.pow(distVector.getX(), 2) + Math.pow(distVector.getY(), 2));
  }

  // Locks the shooter horizontal angle on a given target
  public static double getThetaAngle(Translation2d distanceVector, Pose2d current) {
    return Math.atan2(distanceVector.getY(), distanceVector.getX()) - current.getRotation().getRadians() + Math.PI / 2;
  }

  // Creates a target that is offset by current velocity and estimated flight time
  private static Translation3d predictTargetpose(Translation3d target, double time, ChassisSpeeds velocity) {
    double xEstimate = target.getX() - velocity.vxMetersPerSecond * time;
    double yEstimate = target.getY() - velocity.vyMetersPerSecond * time;
    return new Translation3d(xEstimate, yEstimate, target.getZ());
  }

  // Automatically sets the shooter's baseline target. Need to add a constraint
  // system for if alliance.get no worky.
  public static Translation3d target(Pose2d botPose) {
    Translation3d TARGET = new Translation3d();
    double x = botPose.getX();
    if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
      if (x >= 0 && x < ShooterConstants.XVALS.BATRENCH) {
        TARGET = ShooterConstants.TAGRETS.BLUEHUB;
      } else if (x >= ShooterConstants.XVALS.BATRENCH && x <= ShooterConstants.XVALS.BMTRENCH
          || x >= ShooterConstants.XVALS.RMTRENCH && x <= ShooterConstants.XVALS.RATRENCH) {
        TARGET = new Translation3d(botPose.getX(), botPose.getY(), 10);
      } else if (x > ShooterConstants.XVALS.BMTRENCH && x < ShooterConstants.XVALS.RMTRENCH) {
        if (botPose.getY() >= 4.0) {
          TARGET = ShooterConstants.TAGRETS.BLUEPASSUP;
        } else if (botPose.getY() < 4.0) {
          TARGET = ShooterConstants.TAGRETS.BLUEPASSDOWN;
        }
      } else {
        if (botPose.getY() >= 4.0) {
          TARGET = ShooterConstants.TAGRETS.CENTERUP;
        } else if (botPose.getY() < 4.0) {
          TARGET = ShooterConstants.TAGRETS.CENTERDOWN;
        }
      }
    } else if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
      if (x > ShooterConstants.XVALS.RATRENCH && x < ShooterConstants.XVALS.REDWALL) {
        TARGET = new Translation3d(botPose.getX(), botPose.getY(), 10);
      } else if (x >= ShooterConstants.XVALS.BATRENCH && x <= ShooterConstants.XVALS.BMTRENCH
          || x >= ShooterConstants.XVALS.RMTRENCH && x <= ShooterConstants.XVALS.RATRENCH) {
        TARGET = new Translation3d(botPose.getX(), botPose.getY(), 10);
      } else if (x > ShooterConstants.XVALS.BMTRENCH && x < ShooterConstants.XVALS.RMTRENCH) {
        if (botPose.getY() >= 4.0) {
          TARGET = ShooterConstants.TAGRETS.REDPASSUP;
        } else if (botPose.getY() < 4.0) {
          TARGET = ShooterConstants.TAGRETS.REDPASSDOWN;
        }
      } else {
        if (botPose.getY() >= 4.0) {
          TARGET = ShooterConstants.TAGRETS.CENTERUP;
        } else if (botPose.getY() < 4.0) {
          TARGET = ShooterConstants.TAGRETS.CENTERDOWN;
        }
      }
    } else {
      if (botPose.getY() >= 4.0) {
        TARGET = ShooterConstants.TAGRETS.CENTERUP;
      } else if (botPose.getY() < 4.0) {
        TARGET = ShooterConstants.TAGRETS.CENTERDOWN;
      }
    }
    return TARGET;
  }

  // Actual Calculation of optimal shot, should write to be constrained on a certain error. Math is in the methodology.
  public static ShotInfo getShot(ChassisSpeeds velocity, Translation3d target, Pose2d botPose, int iterations) {
    // Flat ground dist to initial target
    Pose2d targetPose = new Pose2d(target.getX(), target.getY(), null);
    double dist = getDist(distVector(targetPose, botPose));

    // Determine intital azimuth and estimate time of flight
    ShotInfo SHOT = ShooterConstants.shotMap.get(dist);
    SHOT = new ShotInfo(SHOT.exitVel(), SHOT.getVertAngle(), target);
    double time = ShooterConstants.timeMap.get(dist);

    // Set predicted target to initial target
    Translation3d predictedTarget = target;

    for (int i = 0; i < iterations; i++) {
      // Predict where we have to aim based on estimated flight time and ball velocity
      predictedTarget = predictTargetpose(target, time, velocity);

      // Update distance
      dist = botPose.getTranslation().getDistance(predictedTarget.toTranslation2d());

      // Recalculate azimuth and time of flight with new distance
      SHOT = ShooterConstants.shotMap.get(dist);
      SHOT = new ShotInfo(SHOT.exitVel(), SHOT.getVertAngle(), predictedTarget);
      time = ShooterConstants.timeMap.get(dist);
    }
    //This is a protective measure. Only time this would be true is when it is set in the target method, which is when on defense or in the trench.
    if (targetPose.getTranslation() == botPose.getTranslation()){
      SHOT = new ShotInfo(SHOT.exitVel, ShooterConstants.maxVert, predictedTarget);
    }

    //Spit out 'optimal' shot info
    return SHOT;
  }

  public record ShotInfo(double exitVel, double vertAngle, Translation3d target) {
    public ShotInfo(double exitVel, double vertAngle) {
      this(exitVel, vertAngle, ShooterConstants.hubPose);
    }

    public double getZComponent() {
      return exitVel * Math.sin(vertAngle);
    }

    public double getXYComponent() {
      return exitVel * Math.cos(vertAngle);
    }

    public double getMag() {
      return exitVel;
    }

    public double getVertAngle() {
      return vertAngle();
    }

    public Translation3d getTarget() {
      return target();
    }

    public static ShotInfo interpolate(ShotInfo estimate, ShotInfo result, double t) {
      return new ShotInfo(estimate.getMag(),
          MathUtil.interpolate(estimate.getVertAngle(), result.getVertAngle(), t),
          result.getTarget());
    }

  }

  public static ShooterConstants.HUBSTATE isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return HUBSTATE.INACTIVE;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return HUBSTATE.ACTIVE;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return HUBSTATE.SOON;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return HUBSTATE.ACTIVE;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return HUBSTATE.ACTIVE;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return HUBSTATE.ACTIVE;
    } else if(matchTime > 110){
      // Shift 1
      return shift1Active ? HUBSTATE.ACTIVE : HUBSTATE.INACTIVE;
    } else if (matchTime > 105) {
      // Switching, last 5 seconds of Shift 1
      return HUBSTATE.SOON;
    } else if (matchTime > 85) {
      // Shift 2
      return !shift1Active ? HUBSTATE.ACTIVE : HUBSTATE.INACTIVE;
    } else if (matchTime > 80) {
      // Shift 2 switching to Shift 3
      return HUBSTATE.SOON;
    } else if (matchTime > 60) {
      // Shift 3
      return shift1Active ? HUBSTATE.ACTIVE : HUBSTATE.INACTIVE;
    } else if (matchTime > 55) {
      // Shift 3 switching to Shift 4
      return HUBSTATE.SOON;
    } else if (matchTime > 35) {
      // Shift 4
      return !shift1Active ? HUBSTATE.ACTIVE : HUBSTATE.INACTIVE;
    } else if (matchTime > 30) {
      // Shift 4 switching to Endgame
      return HUBSTATE.SOON;
    } else {
      // End game, hub always active.
      return HUBSTATE.ACTIVE;
    }
  }
}
