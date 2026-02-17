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

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;

    private final SparkMaxConfig driveMotorConfig;
    private final SparkMaxConfig turningMotorConfig;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final CANcoderConfiguration config;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, Boolean driveMotorReversed, Boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
        config = new CANcoderConfiguration();


        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        driveMotorConfig = new SparkMaxConfig();
        driveEncoder = driveMotor.getEncoder();


        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        turningMotorConfig = new SparkMaxConfig();

        driveMotorConfig.inverted(driveMotorReversed);
        driveMotorConfig.idleMode(IdleMode.kBrake);

        turningMotorConfig.inverted(turningMotorReversed);
        turningMotorConfig.idleMode(IdleMode.kBrake);
        // turningMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // turningMotorConfig.Feedback.FeedbackRemoteSensorID = absoluteEncoderId;
        // turningMotorConfig.Feedback.RotorToSensorRatio = ModuleConstants.kTurningMotorGearRatio;


        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(config);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


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