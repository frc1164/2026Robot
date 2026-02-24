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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Swerve.SwerveConstants.ModuleConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX drive;
  private final TalonFX turn;

  private final TalonFXConfiguration driveConfig;
  private final TalonFXConfiguration turnConfig;
  
  private final CANcoder canCoder1;
  private final CANcoderConfiguration canCoderConfiguration1;
  
  private final CANcoder canCoder2;
  private final CANcoderConfiguration canCoderConfiguration2;

  private final PIDController pid;

  private final double offset1;
  private final double offset2;

  private final double gear0TeethCount = 70;
  private final double gear1TeethCount = 42;
  private final double gear2TeethCount = 72;
  

  /** Creates a new Shooter. */
  public Shooter() {
    drive = new TalonFX(50);
    turn = new TalonFX(51);

    driveConfig = new TalonFXConfiguration();
    turnConfig = new TalonFXConfiguration();

    // Add Encoder Ids later
    canCoder1 = new CANcoder(0);
    canCoderConfiguration1 = new CANcoderConfiguration();

    canCoder2 = new CANcoder(0);
    canCoderConfiguration2 = new CANcoderConfiguration();

    pid = new PIDController(0.35, 0, 0);
    pid.enableContinuousInput(-Math.PI, Math.PI);

    driveConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    turnConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = 52;
    turnConfig.Feedback.RotorToSensorRatio = 18.75;

    canCoderConfiguration1.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    canCoderConfiguration1.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoder1.getConfigurator().apply(canCoderConfiguration1);

    canCoderConfiguration2.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    canCoderConfiguration2.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoder2.getConfigurator().apply(canCoderConfiguration2);

    turn.getConfigurator().apply(turnConfig);
    drive.getConfigurator().apply(driveConfig);

    offset1 = .156 * 2 * Math.PI;
    offset2 = .156 * 2 * Math.PI;
  }

  private double getGear3Rotation(double r1, double r2) {
    final double d1 = r1 * 360;
    final double d2 = r2 * 360;

    final double t1 = d1 * gear1TeethCount / 360;
    final double t2 = d2 * gear2TeethCount / 360;

    final double bezout = (t1 * 12 * 3 + t2 * 7 * -5) % 504;

    final double totalRot1 = Math.floor(bezout / gear1TeethCount);

    final double rot0 = (totalRot1 + d1 / 360) * gear1TeethCount / gear0TeethCount * 360;

    return rot0 % 360;
  }

  public double getTurningPosition() {
    double gear1Rotation = canCoder1.getPosition().getValueAsDouble() * 360 - offset1;
    double gear2Rotation = canCoder2.getPosition().getValueAsDouble() * 360 - offset2;

    return getGear3Rotation(gear1Rotation, gear2Rotation) * Math.PI / 180;
  }

  public double getDriveVelocity() {
    return drive.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
  }

  public void setDriveVelocity(double speed) {
    drive.set(speed);
  }

  public void runPID(double angle){
    double pidGear0Speed = pid.calculate(getTurningPosition(), angle * Math.PI / 180);
    double pidMotorSpeed = pidGear0Speed * gear0TeethCount / gear1TeethCount;
    turn.set(pidMotorSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle", getTurningPosition());
    // This method will be called once per scheduler run
  }
}
