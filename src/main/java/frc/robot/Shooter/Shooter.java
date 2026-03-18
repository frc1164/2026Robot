// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final SparkMax vert;
  private final SparkMaxConfig vertConfig;

  private final TalonFX shootMot;
  private final TalonFXConfiguration shotConfig;

  private final SparkMax turn;
  private final SparkMaxConfig turnConfig;

  
  @SuppressWarnings("unused")
  private final AbsoluteEncoder absEncoder;
  private final RelativeEncoder relEncoder;

  private final AbsoluteEncoder vertEncoder;
  private final AbsoluteEncoderConfig vertEncoderConfig;

  private final PIDController thetaPID, vertPID, shotPID;

  private final Feeder feeder;

  // private final double gear0TeethCount = 132;
  // private final double gear1TeethCount = 17;
  // private final double gear2TeethCount = 36;

  // private final double n1 = 17; // g1 * n1 (mod g2) = 1
  // private final double n2 = 9; // g2 * n2 (mod g1) = 1
  // private final double lcm = 612; // lcm(g1, g2)

  private double lastSpeed;
  private double currentTheta;

  private static Optional<Alliance> alliance;

  /** Creates a new Shooter. */
  public Shooter(Feeder m_feeder) {

    // Instantiate and configure the pivot
    turn = new SparkMax(51, MotorType.kBrushless);
    turnConfig = new SparkMaxConfig();
    turnConfig.inverted(false);
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.encoder.velocityConversionFactor(1/9 * 36 / 132);
    turnConfig.encoder.positionConversionFactor(1/9 * 36 / 132);
    turn.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    feeder = m_feeder;
    absEncoder = feeder.getAbsoluteEncoder(); // all the configuration logic occurs in Feeder
    relEncoder = turn.getEncoder();


    // Instantiate and configure the hood
    vert = new SparkMax(54, MotorType.kBrushless);
    vertConfig = new SparkMaxConfig();

    vertEncoder = vert.getAbsoluteEncoder();
    vertEncoderConfig = new AbsoluteEncoderConfig();
    vertEncoderConfig.zeroOffset(0.1004375 + 0.75)
        .inverted(false);

    vertConfig.apply(vertEncoderConfig);
    vertConfig.inverted(true).idleMode(IdleMode.kBrake);
    vert.configure(vertConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Instantiate and configure shooter
    shootMot = new TalonFX(55);
    shotConfig = new TalonFXConfiguration();
    shotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootMot.getConfigurator().apply(shotConfig);

    // Instantiate PID's
    thetaPID = new PIDController(0.001, 0, 0);
    vertPID = new PIDController(0.039, 0.00006, 0.0001);
    shotPID = new PIDController(.0001, 0, 0.00003);

    lastSpeed = 0;
    currentTheta = Math.PI / 2; //might be 3/2 pi

    alliance = DriverStation.getAlliance();
  }

  // private double getGear3Rotation(double r1, double r2) {
  //   final double d1 = r1 * 360;
  //   final double d2 = r2 * 360;

  //   final double t1 = d1 * gear1TeethCount / 360;
  //   final double t2 = d2 * gear2TeethCount / 360;

  //   final double bezout = (t1 * gear2TeethCount * n2 + t2 * gear1TeethCount * n1) % lcm;

  //   final double totalRot1 = Math.floor(bezout / gear1TeethCount);

  //   final double rot0 = (totalRot1 + d1 / 360) * gear1TeethCount / gear0TeethCount * 360;

  //   return rot0;
  // }

  // public double getThetaPosition() {
  //   double gear1Rotation = encoder1.getPosition() * 360;
  //   double gear2Rotation = encoder2.getPosition() * 360;

  //   return (getGear3Rotation(gear1Rotation, gear2Rotation) % 360) * Math.PI / 180;
  // }


  public final double getThetaPosition(){
    currentTheta = relEncoder.getPosition();
    return currentTheta * Math.PI * 2;
  }

  public double getPhiPosition() {
    return vertEncoder.getPosition() * 360;
  }

  public void runPhiPID(double degrees) {
    double angle = -(degrees - 85.6) + 90;
    angle = Math.max(91, Math.min(angle, 117));

    double power = vertPID.calculate(getPhiPosition(), angle) + (6 - 90) * 0.00456368213471;

    if (power > 0.25) {
      power = 0.21;
    } else if (power < -0.15) {
      power = -0.17;
    }
    vert.set(power);
    SmartDashboard.putNumber("setpt", angle);
    SmartDashboard.putNumber("location", vertEncoder.getPosition() * 360);
  }

  public void setShotSpeed(double speed) {// 4000rpm
    double PIDoutput = shotPID.calculate(shootMot.getVelocity().getValueAsDouble() * 60, speed);
    double power = PIDoutput + lastSpeed;
    if (power <= 0 || !DriverStation.isTeleopEnabled()) {
      power = 0;
    }
    shootMot.set(power);
    lastSpeed = power;
    SmartDashboard.putNumber("lastSpeed", lastSpeed);
  }

  // Once we know the range of theta, we will have to program in limits to this in
  // a weird way, hopefully we can leave it swapping at 0.
  public void runThetaPID(double radians) {
    double pidMotorSpeed = thetaPID.calculate(getThetaPosition(), radians);
    turn.set(pidMotorSpeed);
  }

  public static Optional<Alliance> getAlliance(){
    return alliance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
