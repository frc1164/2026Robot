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

  private final double gear0TeethCount = 132;
  private final double gear1TeethCount = 17;
  private final double gear2TeethCount = 36;

  // Varience in each of the gears
  private final double sigma1 = 1;
  private final double sigma2 = 1;

  // Classic CRT values
  // (6) Modular multiplicative inverses
  private final double n1 = 17; // g1 * n1 (mod g2) = 1
  private final double n2 = 9; // g2 * n2 (mod g1) = 1

  // (9) Kappa values are all modulo and their inverses that aren't the current index i
  private final double y1 = n2 * gear2TeethCount;  
  private final double y2 = n1 * gear1TeethCount;

  private final double lcm = 612; // lcm(g1, g2)

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

    public double circularDistance(double a, double b, int modulo) {
    // (11) Distance from a to b in a circular context circumfrance of modulo
    return (a - b - modulo * Math.round((a - b) / modulo));
  }

  public double calculateCommonRemainder(double r1, double r2) {
    // (37) Weights of each of the variances
    double w1 = (1 / sigma1 * sigma1) / ((1 / sigma1 * sigma1) + (1 / sigma2 * sigma2));
    double w2 = (1 / sigma2 * sigma2) / ((1 / sigma1 * sigma1) + (1 / sigma2 * sigma2));

    // (26) Decimal values of each of the remainders
    double rc1 = r1 - Math.floor(r1);
    double rc2 = r2 - Math.floor(r2);

    // (39) Set Omega contains L elements where L is amount of moduli
    double o1 = (w1 * rc1 + w2 * rc2 + Math.min(w1, w2)) % 1;
    double o2 = (w1 * rc1 + w2 * rc2 + w1 + w2) % 1;

    // (45) Using elements from Omega, use them to check for where the function is at a minimum
    double distRc1o1 = circularDistance(rc1, o1, 1);
    double distRc1o2 = circularDistance(rc1, o2, 1);
    double distRc2o1 = circularDistance(rc2, o1, 1);
    double distRc2o2 = circularDistance(rc2, o2, 1);

    double rcCandidate1 = w1 * distRc1o1 * distRc1o1 + w2 * distRc2o1 * distRc2o1;
    double rcCandidate2 = w1 * distRc1o2 * distRc1o2 + w2 * distRc2o2 * distRc2o2;

    // (45) return o1 if it was where the function was at its minimum, otherwise o2
    if (rcCandidate1 < rcCandidate2) {
      return o1;
    } else {
      return o2;
    }
  }

  public double calculateMLECRT(double r1, double r2) {
    // (26) Above this it states that rc is significant in the estimation
    double rc = calculateCommonRemainder(r1, r2);

    // (28) Uses the rc to make the whole component much more accurate
    double q1 = Math.round(r1 - rc);
    double q2 = Math.round(r2 - rc);

    // (29) Again with Classic CRT except using the new values
    double n0 = (q1 * y1 + q2 + y2) % lcm;

    // (30) Final equation, since gcd of moduli is 1, just adds rc
    double n = n0 + rc;

    return n;
  }

  public final double getThetaPosition(){ 
    // currentTheta = relEncoder.getPosition();
    // return currentTheta * Math.PI * 2;

    // Feed calculateMLECRT with the values from the 2 encoders we used previously
    // I put the relative equations and their indexes that come from the document at https://www.eecis.udel.edu/~xxia/CRTR.pdf
    // Key requirement for the measurement to be accurate is that the noise must be less than M/4 where M is the gcd of the moduli, in this case is 1. So needs 1/4 of a tooth of accuracy.
    return calculateMLECRT(0, 0);
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

  public void setShotSpeed(double speed) {// 4000rpm to shoot
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
