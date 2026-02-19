// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Swerve.SwerveConstants.ModuleConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX drive;
  private final TalonFX turn;

  private final TalonFXConfiguration driveConfig;
  private final TalonFXConfiguration turnConfig;
  
  private final CANcoder canCoder;
  private final CANcoderConfiguration canCoderConfiguration;
  private final PIDController pid;

  private final double offset;

  /** Creates a new Shooter. */
  public Shooter() {
    drive = new TalonFX(50);
    turn = new TalonFX(51);

    driveConfig = new TalonFXConfiguration();
    turnConfig = new TalonFXConfiguration();

    canCoder = new CANcoder(52);
    canCoderConfiguration = new CANcoderConfiguration();

    pid = new PIDController(0.35, 0, 0);
    pid.enableContinuousInput(-Math.PI, Math.PI);

    driveConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    turnConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = 52;
    turnConfig.Feedback.RotorToSensorRatio = 18.75;

    canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoder.getConfigurator().apply(canCoderConfiguration);

    turn.getConfigurator().apply(turnConfig);
    drive.getConfigurator().apply(driveConfig);

    offset = 0.0;
  }

  public double getTurningPosition() {
    return turn.getPosition().getValueAsDouble() * Math.PI / 180 - offset;
  }

  public double getDriveVelocity() {
    return drive.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
  }

  public void setDriveVelocity(double speed) {
    drive.set(speed);
  }

  public double getAbsoluteEncoderRad() {
    double angle = canCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    angle -= offset;
    return angle * -1.0;
  }

  public void runPID(double angle){
    turn.set(pid.calculate(getTurningPosition(), angle * Math.PI / 180));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
