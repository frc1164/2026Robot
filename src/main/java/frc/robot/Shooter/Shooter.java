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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Swerve.SwerveConstants.ModuleConstants;

public class Shooter extends SubsystemBase {
  // private final SparkMax turnMotor;
  // private final TalonFX drive;
  private final TalonFX turn;

  // private final TalonFXConfiguration driveConfig;
  private final TalonFXConfiguration turnConfig;
  
  private final CANcoder canCoder1;
  private final CANcoderConfiguration canCoderConfiguration1;
  
  // private final CANcoder canCoder2;
  // private final CANcoderConfiguration canCoderConfiguration2;

  private final PIDController pid;

  private final double offset1;
  // private final double offset2;

  private final double gear0TeethCount = 132;
  private final double gear1TeethCount = 17;
  private final double gear2TeethCount = 36;

  private final double n1 = 17; // g1 * n1 (mod g2) = 1
  private final double n2 = 9; // g2 * n2 (mod g1) = 1
  private final double lcm = 612; // lcm(g1, g2)
  

  /** Creates a new Shooter. */
  public Shooter() {
    // turnMotor = new SparkMax(50, MotorType.kBrushless);

    // drive = new TalonFX(50);
    turn = new TalonFX(51);

    // driveConfig = new TalonFXConfiguration();
    turnConfig = new TalonFXConfiguration();

    // Add Encoder Ids later
    canCoder1 = new CANcoder(52);
    canCoderConfiguration1 = new CANcoderConfiguration();

    // canCoder2 = new CANcoder(0);
    // canCoderConfiguration2 = new CANcoderConfiguration();

    pid = new PIDController(0.35 * 2, 0, 0);
    pid.enableContinuousInput(-Math.PI, Math.PI);

    // driveConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    // turnConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    // driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // turnConfig.Feedback.FeedbackRemoteSensorID = 52;
    // turnConfig.Feedback.RotorToSensorRatio = 18.75;

    canCoderConfiguration1.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    canCoderConfiguration1.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfiguration1.MagnetSensor.MagnetOffset = -.1555 * 2 * Math.PI;
    canCoder1.getConfigurator().apply(canCoderConfiguration1);

    // canCoderConfiguration2.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    // canCoderConfiguration2.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // canCoder2.getConfigurator().apply(canCoderConfiguration2);

    // turn.getConfigurator().apply(turnConfig);
    // drive.getConfigurator().apply(driveConfig);

    offset1 = .156 * 2 * Math.PI;
    // offset2 = .156 * 2 * Math.PI;
  }

  private double getGear3Rotation(double r1, double r2) {
    final double d1 = r1 * 360;
    final double d2 = r2 * 360;

    final double t1 = d1 * gear1TeethCount / 360;
    final double t2 = d2 * gear2TeethCount / 360;

    final double bezout = (t1 * gear2TeethCount * n2 + t2 * gear1TeethCount * n1) % lcm;

    final double totalRot1 = Math.floor(bezout / gear1TeethCount);

    final double rot0 = (totalRot1 + d1 / 360) * gear1TeethCount / gear0TeethCount * 360;

    return rot0;
  }

  public double getTurningPosition() {
    double gear1Rotation = canCoder1.getPosition().getValueAsDouble() * 360 - offset1;
    // double gear2Rotation = canCoder2.getPosition().getValueAsDouble() * 360 - offset2;

    // return (getGear3Rotation(gear1Rotation, gear2Rotation) % 360) * Math.PI / 180;
    return canCoder1.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  // public double getDriveVelocity() {
  //   return drive.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
  // }

  // public void setDriveVelocity(double speed) {
  //   drive.set(speed);
  // }

  public void runPID(double angle){//feed this radians
    double gear1Rotation = canCoder1.getPosition().getValueAsDouble() * 360 - offset1;
    // double gear2Rotation = canCoder2.getPosition().getValueAsDouble() * 360 - offset2;

    //PID will not stop running, only recieves updated angles
    

    double pidGear0Speed = pid.calculate(getTurningPosition(), angle * Math.PI / 180);
    // double pidMotorSpeed = pidGear0Speed * gear0TeethCount / gear1TeethCount;
    double pidMotorSpeed = pid.calculate(getTurningPosition(), angle);
    turn.set(pidMotorSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle", getTurningPosition());
    // This method will be called once per scheduler run
  }
}
