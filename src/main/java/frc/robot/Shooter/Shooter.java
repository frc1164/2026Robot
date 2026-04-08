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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Swerve.SwerveSubsystem;

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
  // private final double gear0TeethCount = 132;
  // private final double gear1TeethCount = 17;
  // private final double gear2TeethCount = 36;

  // private final double n1 = 17; // g1 * n1 (mod g2) = 1
  // private final double n2 = 9; // g2 * n2 (mod g1) = 1
  // private final double lcm = 612; // lcm(g1, g2)

  private double lastSpeed;
  private double currentTheta;

  private static Optional<Alliance> alliance;

  private boolean runShooter, blue, active;

  private SwerveSubsystem swerve;
  private Translation3d HUB;

  private Timer turnOnTimer;

  /** Creates a new Shooter. */
  public Shooter(SwerveSubsystem Swerve) {

    // Instantiate and configure the pivot
    turn = new SparkMax(51, MotorType.kBrushless);
    turnConfig = new SparkMaxConfig();
    turnConfig.inverted(false);
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.encoder.velocityConversionFactor(1.0 / 9.0 * 36.0 / 132.0);
    turnConfig.encoder.positionConversionFactor(1.0 / 9.0 * 36.0 / 132.0);
    turn.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

    //Intantiate Swerve
    swerve = Swerve;

    //Initialize Important Variables
    lastSpeed = 0;
    currentTheta = .25 + 4.0/360.0; // might be 3/2 pi
    relEncoder.setPosition(currentTheta);
    alliance = DriverStation.getAlliance();
    runShooter = false;
    blue = Shooter.getAlliance().get() == Alliance.Blue;
    active = false;
    turnOnTimer = new Timer();

    if (blue) {
      HUB = ShooterConstants.TAGRETS.BLUEHUB;
    } else if (!blue) {
      HUB = ShooterConstants.TAGRETS.REDHUB;
    } else {
      HUB = ShooterConstants.TAGRETS.BLUEHUB;
    }
  }

  public final double getThetaPosition() {
    currentTheta = relEncoder.getPosition();
    return currentTheta * 360;
  }

  public double getPhiPosition() {
    return vertEncoder.getPosition() * 360;
  }

  public void runPhiPID(double degrees) {
    double angle = degrees;
    angle = Math.max(94.6, Math.min(angle, 116));

    double power = vertPID.calculate(getPhiPosition(), angle ) + (96.4 - 90) * 0.00456368213471;

    if (power > 0.25) {
      power = 0.21;
    } else if (power < -0.15) {
      power = -0.17;
    }

    if (Double.isNaN(power) == true) {
      power = 0;
      vertPID = new PIDController(0.039, 0.00006, 0.0001);
    }

    if (active){
      vert.set(power);
    } else {
      vert.set(0);
    }
    SmartDashboard.putNumber("setpt", angle);
    SmartDashboard.putNumber("location", vertEncoder.getPosition() * 360);
    SmartDashboard.putNumber("power", power);
  }

  public void setShotSpeed(boolean stop) {// 4000rpm to shoot
    double speed = 3250;
    if (!opposingAlliance()){
      speed = 3250;
    }else{
      speed = 5000;
    }
    double PIDoutput = shotPID.calculate(shootMot.getVelocity().getValueAsDouble() * 60, speed);
    double power = PIDoutput + lastSpeed;
    if (power <= 0) {
      power = 0;
    }
    SmartDashboard.putNumber("PIDoutput", PIDoutput);
    SmartDashboard.putBoolean("ShooterOn", stop);
    lastSpeed = Math.min(power, 1);
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
  public void runThetaPID(double degrees) {
    degrees = ((degrees % 360) + 360) % 360;

    double pidMotorSpeed = thetaPID.calculate(getThetaPosition(), degrees);
    pidMotorSpeed = Math.max(Math.min(pidMotorSpeed, .75), -.75);
    if(active){
      turn.set(pidMotorSpeed);
    }else{
      turn.set(0);
    }
    SmartDashboard.putNumber("MotorOutput", pidMotorSpeed);
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

  public boolean aimingAtHub(){
    if (ShooterCalculator.target(swerve.getPose(), blue) == HUB){
      return true;
    }else{
      return false;
    }
  }

  public boolean opposingAlliance(){
    if (ShooterCalculator.target(swerve.getPose(), blue) == ShooterConstants.TAGRETS.CENTERDOWN || ShooterCalculator.target(swerve.getPose(), blue) == ShooterConstants.TAGRETS.CENTERUP){
      return true;
    } else {
      return false;
    }
  }

  public void resetTimer(){
    turnOnTimer.reset();
  }
  public void startTimer(){
    turnOnTimer.reset();
    turnOnTimer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (runShooter){
      setShotSpeed(false);
    } else {
      setShotSpeed(true);
    }
    SmartDashboard.putNumber("timer", turnOnTimer.get());
    if(turnOnTimer.hasElapsed(.5)){
      active = true;
      turnOnTimer.stop();
    } else {
      active = false;
    }


    SmartDashboard.putNumber("theta", getThetaPosition());
    SmartDashboard.putNumber("ShooterSpeed", shootMot.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("lastSpeed", lastSpeed);
  }
}
