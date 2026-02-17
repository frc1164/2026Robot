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
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 18.75;// 7 / 150;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2.0 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.35;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(24);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), /* Left front */
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), /* Right front */
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), /* Left rear */
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); /* Right rear */

        public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kBackLeftDriveMotorPort = 40;
        public static final int kFrontRightDriveMotorPort = 20;
        public static final int kBackRightDriveMotorPort = 30;

        public static final int kFrontLeftTurningMotorPort = 11;
        public static final int kBackLeftTurningMotorPort = 41;
        public static final int kFrontRightTurningMotorPort = 21;
        public static final int kBackRightTurningMotorPort = 31;

        public static final Boolean kFrontLeftTurningEncoderReversed = Boolean.TRUE;
        public static final Boolean kBackLeftTurningEncoderReversed = Boolean.TRUE;
        public static final Boolean kFrontRightTurningEncoderReversed = Boolean.TRUE;
        public static final Boolean kBackRightTurningEncoderReversed = Boolean.TRUE;

        public static final Boolean kFrontLeftDriveEncoderReversed = Boolean.TRUE;
        public static final Boolean kBackLeftDriveEncoderReversed = Boolean.TRUE;
        public static final Boolean kFrontRightDriveEncoderReversed = Boolean.FALSE;
        public static final Boolean kBackRightDriveEncoderReversed = Boolean.FALSE;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 42;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 22;
        public static final int kBackRightDriveAbsoluteEncoderPort = 32;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;


        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 51.76764  * Math.PI / 180.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (234.9324 - 32.11272) * Math.PI / 180.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = (71.367120 - 16.787) * Math.PI / 180.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = (23.3536+12.24) * Math.PI / 180.0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

 

        public static final double kSLeft = 0.32614;
        public static final double kVLeft = 4.0056;
        public static final double kALeft = 0.33487;

        public static final double kSRight = 0.28932;
        public static final double kVRight = 4.0178;
        public static final double kARight = 0.10801;


        // Drive/Rotation gain
        public static final double kRotGain = 3;
        public static final double kDriveGain = 4.5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPTranslationController = 5;
        public static final double kDTranslationController = 0.075;
        public static final double kPThetaController = 5;
        public static final double kDThetaController = 0.075;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class LimeLightConstants{
        public static final String kLLTags = "limelight-tags";
        public static final String kTagLimelightNetworkTableName = "limelight-tags";
        public static final int kAprilTagPipeline = 0;
    }
}
