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
import edu.wpi.first.wpilibj.DigitalInput;
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

  private boolean gate;
  private DigitalInput initSensor;

  // CRT Constants
  // private final Feeder m_Feeder;

  // private final double gear0TeethCount = 132;
  // private final double gear1TeethCount = 17;
  // private final double gear2TeethCount = 36;

  // // Varience in each of the gears
  // private final double sigma1 = .25;
  // private final double sigma2 = .25;

  // // Classic CRT values
  // // (6) Modular multiplicative inverses
  // private final double n1 = 17; // g1 * n1 (mod g2) = 1
  // private final double n2 = 9; // g2 * n2 (mod g1) = 1

  // // (9) Kappa values are all modulo and their inverses that aren't the current
  // index i
  // private final double y1 = n2 * gear2TeethCount;
  // private final double y2 = n1 * gear1TeethCount;

  // private final double lcm = 612; // lcm(g1, g2)

  /** Creates a new Shooter. */
  public Shooter(SwerveSubsystem Swerve, Feeder feeder) {

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

    // Intantiate Swerve
    swerve = Swerve;

    // Initialize Important Variables
    lastSpeed = 0;
    // currentTheta = .25 + 4.0/360.0; // might be 3/2 pi
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

    // Initialize Feeder for CRT
    // m_Feeder = feeder;

    // Startup Routine for Turret, gate = true means it running
    gate = true;
    initSensor = new DigitalInput(0);
  }

  public final double getThetaPosition() {
    currentTheta = relEncoder.getPosition();
    return currentTheta * 360;
  }
  // public double circularDistance(double a, double b, int modulo) {
  // // (11) Distance from a to b in a circular context circumfrance of modulo
  // return (a - b - modulo * Math.round((a - b) / modulo));
  // }

  // public double calculateCommonRemainder(double r1, double r2) {
  // // (37) Weights of each of the variances
  // double w1 = (1 / (sigma1 * sigma1)) / ((1 / (sigma1 * sigma1)) + (1 / (sigma2
  // * sigma2)));
  // double w2 = (1 / (sigma2 * sigma2)) / ((1 / (sigma1 * sigma1)) + (1 / (sigma2
  // * sigma2)));

  // // (26) Decimal values of each of the remainders
  // double rc1 = r1 - Math.floor(r1);
  // double rc2 = r2 - Math.floor(r2);

  // // (39) Set Omega contains L elements where L is amount of moduli
  // double o1 = (w1 * rc1 + w2 * rc2 + Math.min(w1, w2)) % 1;
  // double o2 = (w1 * rc1 + w2 * rc2 + w1 + w2) % 1;

  // // (45) Using elements from Omega, use them to check for where the function
  // is at a minimum
  // double distRc1o1 = circularDistance(rc1, o1, 1);
  // double distRc1o2 = circularDistance(rc1, o2, 1);
  // double distRc2o1 = circularDistance(rc2, o1, 1);
  // double distRc2o2 = circularDistance(rc2, o2, 1);

  // double rcCandidate1 = w1 * distRc1o1 * distRc1o1 + w2 * distRc2o1 *
  // distRc2o1;
  // double rcCandidate2 = w1 * distRc1o2 * distRc1o2 + w2 * distRc2o2 *
  // distRc2o2;

  // // (45) return o1 if it was where the function was at its minimum, otherwise
  // o2
  // if (rcCandidate1 < rcCandidate2) {
  // return o1;
  // } else {
  // return o2;
  // }
  // }

  // public double calculateMLECRT(double r1, double r2) {
  // // (26) Above this it states that rc is significant in the estimation
  // double rc = calculateCommonRemainder(r1, r2);

  // // (28) Uses the rc to make the whole component much more accurate
  // double q1 = Math.round(r1 - rc);
  // double q2 = Math.round(r2 - rc);

  // // (29) Again with Classic CRT except using the new values
  // double n0 = (q1 * y1 + q2 + y2) % lcm;

  // // (30) Final equation, since gcd of moduli is 1, just adds rc
  // double n = n0 + rc;

  // return n;
  // }

  // public final double getThetaPosition(){
  // // currentTheta = relEncoder.getPosition();
  // // return currentTheta * Math.PI * 2;

  // // Feed calculateMLECRT with the values from the 2 encoders we used
  // previously
  // // I put the relative equations and their indexes that come from the document
  // at https://www.eecis.udel.edu/~xxia/CRTR.pdf
  // // Key requirement for the measurement to be accurate is that the noise must
  // be less than M/4 where M is the gcd of the moduli, in this case is 1. So
  // needs 1/4 of a tooth of accuracy.
  // return calculateMLECRT(m_Feeder.encoderC().getPosition(),
  // m_Feeder.encoderA().getPosition());
  // }

  public double getPhiPosition() {
    return vertEncoder.getPosition() * 360;
  }

  public void runPhiPID(double degrees) {
    double angle = degrees;
    angle = Math.max(94.6, Math.min(angle, 116));

    double power = vertPID.calculate(getPhiPosition(), angle) + (96.4 - 90) * 0.00456368213471;

    if (power > 0.25) {
      power = 0.21;
    } else if (power < -0.15) {
      power = -0.17;
    }

    if (Double.isNaN(power) == true) {
      power = 0;
      vertPID = new PIDController(0.039, 0.00006, 0.0001);
    }

    if (active) {
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
    if (!opposingAlliance()) {
      speed = 3250;
    } else {
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

    if (!gate) {
      turn.set(.1);
    } else {
      double pidMotorSpeed = thetaPID.calculate(getThetaPosition(), degrees);
      pidMotorSpeed = Math.max(Math.min(pidMotorSpeed, .75), -.75);
      if (active) {
        turn.set(pidMotorSpeed);
      } else {
        turn.set(0);
      }
      SmartDashboard.putNumber("MotorOutput", pidMotorSpeed);
    }
  }

  private void initialize() {
    if (!gate) {
      if (initSensor.get()) {
        relEncoder.setPosition(0); // arbitrary number we find sensor to be at
        gate = false;
      }
    }
  }

  public static Optional<Alliance> getAlliance() {
    return alliance;
  }

  public void resetLastSpeed() {
    lastSpeed = 0;
  }

  public void shooterGoShoot(boolean makeItGo) {
    runShooter = makeItGo;
  }

  public double shooterSpeed() {
    return shootMot.getVelocity().getValueAsDouble() * 60;
  }

  public boolean aimingAtHub() {
    if (ShooterCalculator.target(swerve.getPose(), blue) == HUB) {
      return true;
    } else {
      return false;
    }
  }

  public boolean aimingAtSelf() {
    if (ShooterCalculator.target(swerve.getPose(), blue) == new Translation3d(swerve.getPose().getTranslation())) {
      return true;
    } else {
      return false;
    }
  }

  public boolean opposingAlliance() {
    if (ShooterCalculator.target(swerve.getPose(), blue) == ShooterConstants.TAGRETS.CENTERDOWN
        || ShooterCalculator.target(swerve.getPose(), blue) == ShooterConstants.TAGRETS.CENTERUP) {
      return true;
    } else {
      return false;
    }
  }

  public void resetTimer() {
    turnOnTimer.reset();
  }

  public void startTimer() {
    turnOnTimer.reset();
    turnOnTimer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (runShooter) {
      setShotSpeed(false);
    } else {
      setShotSpeed(true);
    }
    SmartDashboard.putNumber("timer", turnOnTimer.get());
    if (turnOnTimer.hasElapsed(.5)) {
      active = true;
      turnOnTimer.stop();
    } else {
      active = false;
    }

    initialize();

    SmartDashboard.putNumber("theta", getThetaPosition());
    SmartDashboard.putBoolean("Initialization", gate);
    // SmartDashboard.putNumber("bigGear", m_Feeder.encoderA().getPosition()); //
    // CRT Readouts
    // SmartDashboard.putNumber("smallGear", m_Feeder.encoderC().getPosition());
    // //CRT Readouts
    SmartDashboard.putNumber("ShooterSpeed", shootMot.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("lastSpeed", lastSpeed);
  }
}
