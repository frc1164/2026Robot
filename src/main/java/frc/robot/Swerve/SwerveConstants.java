// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SwerveConstants {

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDeadband = 0.15;
  }
  
    public static final class ModuleConstants {

        // Mechanical specs of each module. Can find of site where they were bought. Make sure they match which model you buy
        // These SHOULD be for Daedalus
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 18.75;// 7 / 150;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2.0 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;

        // This is how aggressively the turn motors move to their desired angle
        // Change with caution but definitely tune for new chassis
        public static final double kPTurning = 0.35;
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(24);
    
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(24);

        // Creates a swerve object where each module is on a 2d plane where (0,0) is robot center
        // Can totally make other quadrilaterals here, the SwerveDriveKinematics class handles the math for trapezoids etc
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), /* Left front */
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), /* Right front */
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), /* Left rear */
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); /* Right rear */

        // Drive motor CAN ID's, I like to separate each module by 10's place
        public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kBackLeftDriveMotorPort = 40;
        public static final int kFrontRightDriveMotorPort = 20;
        public static final int kBackRightDriveMotorPort = 30;

        // Turn motor CAN ID's
        public static final int kFrontLeftTurningMotorPort = 11;
        public static final int kBackLeftTurningMotorPort = 41;
        public static final int kFrontRightTurningMotorPort = 21;
        public static final int kBackRightTurningMotorPort = 31;

        // Inversion booleans for turn motors, invert these if the turn motors go crazy on startup
        public static final Boolean kFrontLeftTurningEncoderReversed = Boolean.TRUE;
        public static final Boolean kBackLeftTurningEncoderReversed = Boolean.TRUE;
        public static final Boolean kFrontRightTurningEncoderReversed = Boolean.TRUE;
        public static final Boolean kBackRightTurningEncoderReversed = Boolean.TRUE;

        // Inversion booleans for drive motors, invert these if a wheel(s) spinning backward
        public static final Boolean kFrontLeftDriveEncoderReversed = Boolean.TRUE;
        public static final Boolean kBackLeftDriveEncoderReversed = Boolean.TRUE;
        public static final Boolean kFrontRightDriveEncoderReversed = Boolean.FALSE;
        public static final Boolean kBackRightDriveEncoderReversed = Boolean.FALSE;

        // CANCODER CAN ID's
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 42;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 22;
        public static final int kBackRightDriveAbsoluteEncoderPort = 32;

        // Inversion booleans for CANCODERS. These SHOULDN'T need to ever change
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // Offsets for CANCODERS. One of the first things you should do when configuring new swerve is finding these
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = (167.695-90) * Math.PI/180.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (45.0-90) * Math.PI/180.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = (284.15-90) * Math.PI/180.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = (245.25-90) * Math.PI/180.0;

        // These are used in the SwerveModuleState algo, if you have a game like REEFSCAPE with little travel, consider lowering this
        // Acts kinda like a scalar and is a clamper on robot speed. See SwerveSubsystem line 296
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

        // Leave this constant, trust me
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        // These should also remain unchanged unless for some reason you want to change the rotational acceleration
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

 
        // Feedforward constants from SYSID
        public static final double kSLeft = 0.32614;
        public static final double kVLeft = 4.0056;
        public static final double kALeft = 0.33487;

        public static final double kSRight = 0.28932;
        public static final double kVRight = 4.0178;
        public static final double kARight = 0.10801;


        // Used in SwerveJoystickCommand power curves
        public static final double kRotGain = 3;
        public static final double kDriveGain = 4.5;
    }

    public static final class AutoConstants {
        // PathPlanner configuration, change in accordance to how much you trust auto routine accuracy
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

        // Be VERY careful changing these and DO NOT ADD INTEGRAL TERM- they do two things
        // Control movement across pathplanner paths and control how aggressively the bot will adhere to said lines
        public static final double kPTranslationController = 5;
        public static final double kDTranslationController = 0.075;
        public static final double kPThetaController = 5;
        public static final double kDThetaController = 0.075;

        // Weird stuff to make a Profiled PID for auto that affects acceleration curves... ask Gazeley about Trapezoidal PID's he loves them
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class LimeLightConstants{
        // May have to add more if you decide to use multiple LL's or 1 LL for multiple things
        // I never did the latter but you can swap between 'pipelines' to use different functions of LL's like obj recognition and april tags
        // Can find the pipeline thing in the top of the limelight webclient to manualy switch and configure
        public static final String kLLTags = "limelight-tags";
        public static final String kTagLimelightNetworkTableName = "limelight-tags";
        public static final int kAprilTagPipeline = 0;
    }
}
