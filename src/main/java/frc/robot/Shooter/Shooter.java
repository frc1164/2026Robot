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
  // private final AbsoluteEncoder absEncoder;
  private final RelativeEncoder relEncoder;

  private final AbsoluteEncoder vertEncoder;
  private final AbsoluteEncoderConfig vertEncoderConfig;

  private final PIDController thetaPID;
  private PIDController vertPID;
  private final PIDController shotPID;

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

  private boolean runShooter;

  /** Creates a new Shooter. */
  public Shooter(Feeder m_feeder) {

    // Instantiate and configure the pivot
    turn = new SparkMax(51, MotorType.kBrushless);
    turnConfig = new SparkMaxConfig();
    turnConfig.inverted(false);
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.encoder.velocityConversionFactor(1.0 / 9.0 * 36.0 / 132.0);
    turnConfig.encoder.positionConversionFactor(1.0 / 9.0 * 36.0 / 132.0);
    turn.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    feeder = m_feeder;
    // absEncoder = feeder.getAbsoluteEncoder(); // all the configuration logic
    // occurs in Feeder
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
    shotConfig.TorqueCurrent.PeakForwardTorqueCurrent = 20;
    shootMot.getConfigurator().apply(shotConfig);

    // Instantiate PID's
    thetaPID = new PIDController(0.02, 0, 0);
    vertPID = new PIDController(0.039, 0.00006, 0.0001);
    shotPID = new PIDController(.0002, 0, 0.00003);


    //Initialize Important Variables
    lastSpeed = 0;
    currentTheta = -.25; // might be 3/2 pi
    relEncoder.setPosition(currentTheta);
    alliance = DriverStation.getAlliance();
    runShooter = false;
  }

  public final double getThetaPosition() {
    currentTheta = relEncoder.getPosition();
    return currentTheta * Math.PI * 2;
  }

  public double getPhiPosition() {
    return vertEncoder.getPosition() * 360;
  }

  public void runPhiPID(double degrees) {
    // degrees = Math.max(Math.min(degrees, 81), 57);
    // double angle = -(degrees - 85.6) + 90;
    double angle = degrees;
    angle = Math.max(94.6, Math.min(angle, 116));

    double power = vertPID.calculate(getPhiPosition(), 100 ) + (96.4 - 90) * 0.00456368213471;

    if (power > 0.25) {
      power = 0.21;
    } else if (power < -0.15) {
      power = -0.17;
    }

    if (Double.isNaN(power) == true) {
      power = 0;
      vertPID = new PIDController(0.039, 0.00006, 0.0001);
    }
    // if(getPhiPosition() < 92 && power > 0) {
    // power = 0;
    // }
    // if(getPhiPosition() > 115 && power < 0) {
    // power = 0;
    // }

    vert.set(power);
    SmartDashboard.putNumber("setpt", angle);
    SmartDashboard.putNumber("location", vertEncoder.getPosition() * 360);
    SmartDashboard.putNumber("power", power);
  }

  public void setShotSpeed(double speed, boolean stop) {// 4000rpm to shoot
    double PIDoutput = shotPID.calculate(shootMot.getVelocity().getValueAsDouble() * 60, speed);
    double power = PIDoutput + lastSpeed;
    if (power <= 0 || !DriverStation.isTeleopEnabled()) {
      power = 0;
    }
    SmartDashboard.putNumber("PIDoutput", PIDoutput);
    lastSpeed = power;
    // lastSpeed = Math.max(0,Math.min(2, lastSpeed));
    if (stop) {
      shootMot.set(0);
      resetLastSpeed();
    } else {
      shootMot.set(power);
    }
  }

  // Once we know the range of theta, we will have to program in limits to this in
  // a weird way, hopefully we can leave it swapping at 0.
  public void runThetaPID(double radians) {
    double pidMotorSpeed = thetaPID.calculate(getThetaPosition() * Math.PI / 180, 0);
    SmartDashboard.putNumber("turnPower", pidMotorSpeed);
    turn.set(pidMotorSpeed);
  }

  public static Optional<Alliance> getAlliance() {
    return alliance;
  }

  public void resetLastSpeed() {
    lastSpeed = 0;
  }

  public void shooterGoShoot(boolean makeItGo){
    runShooter = makeItGo;
  }

  public double shooterSpeed(){
    return shootMot.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (runShooter){
      setShotSpeed(4000, false);
    } else {
      setShotSpeed(4000, true);
    }

    SmartDashboard.putNumber("theta", getThetaPosition());
    SmartDashboard.putNumber("ShooterSpeed", shootMot.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("lastSpeed", lastSpeed);
  }
}
