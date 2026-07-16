package frc.robot.Swerve;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Swerve.SwerveConstants.ModuleConstants;


// This serves as a class to define functionality and constructor for swerve modules. There will be 4 instances of this is SwerveSubsystem, 1 for each module.
public class SwerveModule {

    // Motors
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    // Relative encoder for odometry
    private final RelativeEncoder driveEncoder;

    // Configs to handle inversions, offsets, idlemodes, gear ratios, should be defined in SwerveConstants
    private final SparkMaxConfig driveMotorConfig;
    private final SparkMaxConfig turningMotorConfig;

    // PID for turning motor
    private final PIDController turningPidController;

    // CANCODER + config to handle offset, inversions, gear ratios, all defined in SwerveConstants
    private final CANcoder absoluteEncoder;
    private final CANcoderConfiguration config;


    // Guess what these do.
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, Boolean driveMotorReversed, Boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        // Second arg in CANCODER constructor no longer needed and might NEED to be removed for new computer that replaces RoboRio in 2027 season
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
        config = new CANcoderConfiguration();

        // Obviously make sure this is consistent with the motor type youre using
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        driveMotorConfig = new SparkMaxConfig();
        driveEncoder = driveMotor.getEncoder();

    
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        turningMotorConfig = new SparkMaxConfig();

        // These all get configured in constants and pulled into SwerveSubsystem
        driveMotorConfig.inverted(driveMotorReversed)
                        .idleMode(IdleMode.kBrake);

        turningMotorConfig.inverted(turningMotorReversed)
                          .idleMode(IdleMode.kBrake)
                          .encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
                          .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        // For CTRE/Kraken turn motors
        // turningMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // turningMotorConfig.Feedback.FeedbackRemoteSensorID = absoluteEncoderId;
        // turningMotorConfig.Feedback.RotorToSensorRatio = ModuleConstants.kTurningMotorGearRatio;

        // Discontinuity points are points of abiguity/zero points in the absolute encoders range. 1 or 0 mean full rotation and .5 means half rotation.
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(config);

        // PID is just a P controller because it works
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);

        //This just makes the PID work like the sensor discontinuity point where 180, 0, and -180 are all the same thing
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        // Apply configs to motors. 
        // ResetSafeParams basically only matters if you change any in config
        // PersistParams is literally just a save button.
        turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // This just good to do
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return absoluteEncoder.getPosition().getValueAsDouble() * 2 * Math.PI - absoluteEncoderOffsetRad;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return absoluteEncoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }

    /*
     * Returns a double from -pi to pi.
     */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state, SimpleMotorFeedforward feedforward) {
        if (Math.abs(feedforward.calculate(state.speedMetersPerSecond)) < 0.1) {
            stop();
            return;
        }
        state.optimize(getState().angle);
        driveMotor.setVoltage(feedforward.calculate(state.speedMetersPerSecond));
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}