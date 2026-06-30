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

  // Flat ground vector from shooter to target
  public static Translation2d distVector(Pose2d target, Pose2d currentRobotPose) {
    double xDist, yDist;

    // Takes distance from current target to center of bot
    double botDiffX = target.getX() - currentRobotPose.getX();
    double botDiffY = target.getY() - currentRobotPose.getY();

    // Uses trigonometry to find shooter relative to target
    xDist = botDiffX - ShooterConstants.SHOOTEROFFSETS.translation
        * (Math.cos(ShooterConstants.SHOOTEROFFSETS.theta + currentRobotPose.getRotation().getRadians()));
    SmartDashboard.putNumber("xDist", xDist);

    yDist = botDiffY - ShooterConstants.SHOOTEROFFSETS.translation
        * (Math.sin(ShooterConstants.SHOOTEROFFSETS.theta + currentRobotPose.getRotation().getRadians()));
    SmartDashboard.putNumber("yDist", yDist);

    // Outputs a 2d vector
    return new Translation2d(xDist, yDist);
  }

  // Pythagorean theorem applied to given 2d vector
  private static double getDist(Translation2d distVector) {
    double distance = Math.sqrt(Math.pow(distVector.getX(), 2) + Math.pow(distVector.getY(), 2));
    SmartDashboard.putNumber("dist to target", distance);
    return distance;
  }

  // Locks the shooter horizontal angle on a given target
  public static double getThetaAngle(Translation2d distanceVector, Pose2d current) {
    return Math.atan2(distanceVector.getY(), distanceVector.getX()) - current.getRotation().getRadians() + Math.PI;
  }

  // Creates a target that is offset by current velocity and estimated flight time
  private static Translation3d predictTargetpose(Translation3d target, double time, ChassisSpeeds velocity) {
    double xEstimate = target.getX() - velocity.vxMetersPerSecond * time * 62.2857;
    double yEstimate = target.getY() - velocity.vyMetersPerSecond * time * 62.2857;
    /* This 62.2857 seems relatively arbitrary but fixed an issue in a very time sensitive moment.
     * It is the conversion factor between the Swerve speed outputs and the real speed outputs. */
    SmartDashboard.putNumber("fieldEstimatedSpeed", Math.sqrt(Math.pow(velocity.vyMetersPerSecond, 2) + Math.pow(velocity.vxMetersPerSecond, 2)));
    return new Translation3d(xEstimate, yEstimate, target.getZ());
  }

  // Automatically sets the shooter's baseline target.
  /*
   * This is a large and ugly block of nested if and if else statements. While I
   * dislike that I had to do it this way,
   * switch statements do not provide the necessary functionality: nesting.
   * This block serves the purpose of translating the robot's current pose into a
   * target for the turret to aim for.
   * The outermost if statement handles which side/alliance the bot is on. Within
   * that, the entire field is separated by x coordinate into 4 zones with
   * different targets.
   * Two of those zones also must be separated by y coordinate.
   */
  public static Translation3d target(Pose2d botPose, boolean blue) {
    Translation3d TARGET = new Translation3d();
    double x = botPose.getX();
    if (blue) {
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
    } else if (!blue) {
      if (x > ShooterConstants.XVALS.RATRENCH && x < ShooterConstants.XVALS.REDWALL) {
        TARGET = ShooterConstants.TAGRETS.REDHUB;
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
    } else { // In case for some reason the entire pose estimation system fails this should
             // have a reasonable output
      if (botPose.getY() >= 4.0) {
        TARGET = ShooterConstants.TAGRETS.CENTERUP;
      } else if (botPose.getY() < 4.0) {
        TARGET = ShooterConstants.TAGRETS.CENTERDOWN;
      }
    }
    SmartDashboard.putString("TARGET", TARGET.toString());
    return TARGET;
  }

  // Actual Calculation of optimal shot, should write to be constrained on a
  // certain error. Math is in the document.
  public static ShotInfo getShot(ChassisSpeeds velocity, Translation3d target, Pose2d botPose, int iterations) {
    // Flat ground dist to initial target
    Pose2d targetPose = new Pose2d(target.getX(), target.getY(), null);
    Translation2d distVect = distVector(targetPose, botPose);
    double dist = getDist(distVect);

    // Determine intital azimuth and estimate time of flight
    ShotInfo SHOT = ShooterConstants.shotMap.get(dist);
    SHOT = new ShotInfo(SHOT.exitVel(), SHOT.getVertAngle(), target);
    double time = ShooterConstants.timeMap.get(dist);

    // Set predicted target to initial target
    Translation3d predictedTarget = target;
    int i = 0;

    // Originally I tried a while loop and a function for error so I could guarantee
    // a certain degree of precision.
    // That behaved badly and I instead went with this 5 iteration for-loop that
    // gets 6-12 cm of error at a distance of 4 meters
    for (i = 0; i < 5; i++) {
      // Predict where we have to aim based on estimated flight time and ball velocity
      predictedTarget = predictTargetpose(target, time, velocity);

      // Update distance
      dist = botPose.getTranslation().getDistance(predictedTarget.toTranslation2d());

      // Recalculate azimuth and time of flight with new distance
      SHOT = ShooterConstants.shotMap.get(dist);
      SHOT = new ShotInfo(SHOT.exitVel(), SHOT.getVertAngle(), predictedTarget);
      time = ShooterConstants.timeMap.get(dist);
    }
    // This is a protective measure. Only time this would be true is when it is set
    // in the target method, which is when on defense or in the trench.
    // In implementation it will force the turret into a 'safe mode'
    if (targetPose.getTranslation() == botPose.getTranslation()) {
      SHOT = new ShotInfo(SHOT.exitVel, ShooterConstants.maxVert, predictedTarget);
    }

    // Spit out 'optimal' shot info
    return SHOT;
  }

  // Data class that contains all the necessary values to operate the shooter. See
  // implementation here and in AimCommand.java.
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

  // Informational method called to determine scoring mode and other game data
  public static ShooterConstants.HUBSTATE isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return HUBSTATE.INACTIVE;
    }
    // Hub is always enabled in autonomous period.
    if (DriverStation.isAutonomousEnabled()) {
      return HUBSTATE.ACTIVE;
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

    // Sections time remaining in match, in seconds
    if (matchTime > 130) {
      // Transition shift, hub is active.
      return HUBSTATE.ACTIVE;
    } else if (matchTime > 110) {
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
