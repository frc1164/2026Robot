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

  private final RelativeEncoder relEncoder;

  private final AbsoluteEncoder vertEncoder;
  private final AbsoluteEncoderConfig vertEncoderConfig;

  private final PIDController thetaPID;
  private PIDController vertPID;
  private final PIDController shotPID;

  private double lastSpeed;
  private double currentTheta;

  private static Optional<Alliance> alliance;

  private boolean runShooter, blue, active;

  private SwerveSubsystem swerve;
  private Translation3d HUB;

  private Timer turnOnTimer;

  private boolean gate;
  private DigitalInput initSensor;

  /** Creates a new Shooter. */
  public Shooter(SwerveSubsystem Swerve, Feeder feeder) {

    // Instantiate and configure the pivot
    turn = new SparkMax(51, MotorType.kBrushless);
    turnConfig = new SparkMaxConfig();
    turnConfig.inverted(false);
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.encoder.velocityConversionFactor(1.0 / 9.0 * 36.0 / 132.0); // These numbers are the gear ratio between this sensor and the mechanism it reads.
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
    vertEncoderConfig.zeroOffset(0.8504375) // tested offset of a magnet sensor
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

    // Instantiate Swerve
    swerve = Swerve;

    // Initialize Important Variables
    lastSpeed = 0;
    currentTheta = .25 + 4.0/360.0; //<- this is a manual startup position while we fix the initialization routine
    relEncoder.setPosition(currentTheta);
    alliance = DriverStation.getAlliance();
    runShooter = false;
    blue = Shooter.getAlliance().get() == Alliance.Blue;
    active = false;
    turnOnTimer = new Timer();

    // Ensure aimiing a tthe correct team's scoring zone
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

  // For debuggiing
  public double getPhiPosition() {
    return vertEncoder.getPosition() * 360;
  }

  // Controls the vertical launch angle of shots
  public void runPhiPID(double degrees) {
    // Clamps setpoint to just inside physical limits
    double angle = Math.max(94.6, Math.min(degrees, 116));

    /*
     * Find target velocity from PID controller
     * There is an elastic band on this mechanism which makes it nonlinear
     * PID controllers dislike nonlinear systems so we add a feedforward from the
     * force equation of an elastic/spring
     */
    double power = vertPID.calculate(getPhiPosition(), angle) + (angle - 90) * 0.00456368213471;
    // 96.4 is a safe value to use instead of angle in feedforward when on bumpy terrain, which the controller dislikes

    // Output clamping that prevents mechaniism from moving too fast
    if (power > 0.25) {
      power = 0.21;
    } else if (power < -0.15) {
      power = -0.17;
    }

    // Power can sometimes spit NaN on startup, which angers the PID
    // Solve by turning NaN to 0 and then reinstantiating the PID
    if (Double.isNaN(power) == true) {
      power = 0;
      vertPID = new PIDController(0.039, 0.00006, 0.0001);
    }

    // Prevents the turret from moving when the intake is retracted, due to mechanical overlap
    if (active) {
      vert.set(power);
    } else {
      vert.set(0);
    }

  }

  // A PID controller uses velocity to achieve position.
  // This is the derivative of a PID controller which uses acceleration to achieve velocity
  // This is necessary because every time a shot flies, the shooter wheel is slowed, so a correction must be applied
  public void setShotSpeed(boolean stop) {
    double speed = 3250; // Standard wheel speed when scoring or passing

    if (!opposingAlliance()) {
      speed = 3250;
    } else {
      speed = 5000; // "Stealing", incerases shooter wheel speed to increase range
    }

    // Amount of increased speed from last iteration, acts like an integral term
    double PIDoutput = shotPID.calculate(shootMot.getVelocity().getValueAsDouble() * 60, speed); // rps * 60 = rpm

    // Adds 'integral term' to existing speed
    double power = PIDoutput + lastSpeed;

    // Don't want to run shooter wheel backwards
    if (power <= 0) {
      power = 0;
    }

    // Outputs are in duty cycle, clamps to 1 to make cleaner and not confuse the PID controller
    lastSpeed = Math.min(power, 1);

    // Status output for drivers
    SmartDashboard.putBoolean("ShooterOn", !stop);

    if (stop) {
      shootMot.set(0);
      resetLastSpeed(); // Prevents integral buildup
    } else {
      shootMot.set(power);
    }
  }

  // Operates the PID Controller that handles turret rotation
  public void runThetaPID(double degrees) {
    /*
     * Due to wiring constraints, the full range of the turret is 360 +- 10 degrees
     * That means that we need to limit commanded positions to [0, 360)
     * This line acts as a modulus that keeps values in range, ex. -3 becomes 357
     * This prevents overrun as any value that would cause it would make the turret
     * spin the long way to get there
     */
    degrees = ((degrees % 360) + 360) % 360;

    if (!gate) {
      turn.set(.1); // Forces safe movement until initialized
    } else {
      double pidMotorSpeed = thetaPID.calculate(getThetaPosition(), degrees); // Uses the PID Controller to output best velocity to reach desired position
      pidMotorSpeed = Math.max(Math.min(pidMotorSpeed, .75), -.75); // Clamps maximum speed to a 75% duty cycle
      if (active) {
        turn.set(pidMotorSpeed);
      } else {
        turn.set(0); // shutoff when intake mechanism is retracted
      }
    }
  }

  // Used to pull a single reading off turret initialization sensor, which is placed in an arbitrary point
  private void initialize() {
    if (!gate) {
      if (initSensor.get()) {
        relEncoder.setPosition(0); // arbitrary number we find sensor to be at along the turret's range
        gate = true;
      }
    }
  }

  // Pulls relevant match data
  public static Optional<Alliance> getAlliance() {
    return alliance;
  }

  // For debugging and preventing integral buildup in derived PID controller
  public void resetLastSpeed() {
    lastSpeed = 0;
  }

  // For external calls
  public void shooterGoShoot(boolean makeItGo) {
    runShooter = makeItGo;
  }

  // For debugging
  public double shooterSpeed() {
    return shootMot.getVelocity().getValueAsDouble() * 60;
  }

  // Used in autonomous to trigger scoring
  public boolean aimingAtHub() {
    if (ShooterCalculator.target(swerve.getPose(), blue) == HUB) {
      return true;
    } else {
      return false;
    }
  }

  // Used to implement a "safe mode" that prevents the turret from being hit by an obstacle
  public boolean aimingAtSelf() {
    if (ShooterCalculator.target(swerve.getPose(), blue) == new Translation3d(swerve.getPose().getTranslation())) {
      return true;
    } else {
      return false;
    }
  }

  // Informs 'stealing mode'
  public boolean opposingAlliance() {
    if (ShooterCalculator.target(swerve.getPose(), blue) == ShooterConstants.TAGRETS.CENTERDOWN
        || ShooterCalculator.target(swerve.getPose(), blue) == ShooterConstants.TAGRETS.CENTERUP) {
      return true;
    } else {
      return false;
    }
  }

  // Methods for the timer in periodic
  public void resetTimer() {
    turnOnTimer.reset();
  }

  public void startTimer() {
    turnOnTimer.reset();
    turnOnTimer.start();
  }

  @Override
  public void periodic() {
    if (runShooter) {
      setShotSpeed(false);
    } else {
      setShotSpeed(true);
    }

    // Adds a delay on startup to the turret that prevents it from hitting the intake as it deploys
    if (turnOnTimer.hasElapsed(.5)) {
      active = true;
      turnOnTimer.stop();
    } else {
      active = false;
    }

    initialize();
    // Debugging Readouts
    // SmartDashboard.putNumber("theta", getThetaPosition());
    // SmartDashboard.putBoolean("Initialization", gate);
    // SmartDashboard.putNumber("ShooterSpeed",
    // shootMot.getVelocity().getValueAsDouble() * 60);
    // SmartDashboard.putNumber("lastSpeed", lastSpeed);

    // CRT Readouts
    // SmartDashboard.putNumber("bigGear", m_Feeder.encoderA().getPosition()); //
    // SmartDashboard.putNumber("smallGear", m_Feeder.encoderC().getPosition());
  }

  /*
   * 
   * //Below is code for an in-progress attempt at using Chinese Remainder Theorem
   * with two absolute encoders to read the position of the turret
   * 
   * //This system would eliminate the need for an initialization period and
   * SHOULD be more stable than a single relative encoder
   * 
   * 
   * //CRT Constants and objects
   * private final Feeder m_Feeder;
   * 
   * private final double gear0TeethCount = 132;
   * private final double gear1TeethCount = 17;
   * private final double gear2TeethCount = 36;
   * 
   * private final double n1 = 17; // g1 * n1 (mod g2) = 1
   * private final double n2 = 9; // g2 * n2 (mod g1) = 1
   * private final double lcm = 612; // lcm(g1, g2)
   * 
   * private final double gear0TeethCount = 132;
   * private final double gear1TeethCount = 17;
   * private final double gear2TeethCount = 36;
   * 
   * // Variance in each of the gears
   * private final double sigma1 = .25;
   * private final double sigma2 = .25;
   * 
   * // Classic CRT values
   * // (6) Modular multiplicative inverses
   * private final double n1 = 17; // g1 * n1 (mod g2) = 1
   * private final double n2 = 9; // g2 * n2 (mod g1) = 1
   * 
   * // (9) Kappa values are all modulo and their inverses that aren't the current
   * index i
   * private final double y1 = n2 * gear2TeethCount;
   * private final double y2 = n1 * gear1TeethCount;
   * 
   * private final double lcm = 612; // lcm(g1, g2)
   * 
   * public double circularDistance(double a, double b, int modulo) {
   * // (11) Distance from a to b in a circular context circumfrance of modulo
   * return (a - b - modulo * Math.round((a - b) / modulo));
   * }
   * 
   * public double calculateCommonRemainder(double r1, double r2) {
   * // (37) Weights of each of the variances
   * double w1 = (1 / (sigma1 * sigma1)) / ((1 / (sigma1 * sigma1)) + (1 / (sigma2
   * sigma2)));
   * double w2 = (1 / (sigma2 * sigma2)) / ((1 / (sigma1 * sigma1)) + (1 / (sigma2
   * sigma2)));
   * 
   * // (26) Decimal values of each of the remainders
   * double rc1 = r1 - Math.floor(r1);
   * double rc2 = r2 - Math.floor(r2);
   * 
   * // (39) Set Omega contains L elements where L is amount of moduli
   * double o1 = (w1 * rc1 + w2 * rc2 + Math.min(w1, w2)) % 1;
   * double o2 = (w1 * rc1 + w2 * rc2 + w1 + w2) % 1;
   * 
   * // (45) Using elements from Omega, use them to check for where the function
   * is at a minimum
   * double distRc1o1 = circularDistance(rc1, o1, 1);
   * double distRc1o2 = circularDistance(rc1, o2, 1);
   * double distRc2o1 = circularDistance(rc2, o1, 1);
   * double distRc2o2 = circularDistance(rc2, o2, 1);
   * 
   * double rcCandidate1 = w1 * distRc1o1 * distRc1o1 + w2 * distRc2o1 *
   * distRc2o1;
   * double rcCandidate2 = w1 * distRc1o2 * distRc1o2 + w2 * distRc2o2 *
   * distRc2o2;
   * 
   * // (45) return o1 if it was where the function was at its minimum, otherwise
   * o2
   * if (rcCandidate1 < rcCandidate2) {
   * return o1;
   * } else {
   * return o2;
   * }
   * }
   * 
   * public double calculateMLECRT(double r1, double r2) {
   * // (26) Above this it states that rc is significant in the estimation
   * double rc = calculateCommonRemainder(r1, r2);
   * 
   * // (28) Uses the rc to make the whole component much more accurate
   * double q1 = Math.round(r1 - rc);
   * double q2 = Math.round(r2 - rc);
   * 
   * // (29) Again with Classic CRT except using the new values
   * double n0 = (q1 * y1 + q2 + y2) % lcm;
   * 
   * // (30) Final equation, since gcd of moduli is 1, just adds rc
   * double n = n0 + rc;
   * 
   * return n;
   * }
   * 
   * public final double getThetaPosition(){
   * // currentTheta = relEncoder.getPosition();
   * // return currentTheta * Math.PI * 2;
   * 
   * // Feed calculateMLECRT with the values from the 2 encoders we used
   * previously
   * // I put the relative equations and their indexes that come from the document
   * at https://www.eecis.udel.edu/~xxia/CRTR.pdf
   * // Key requirement for the measurement to be accurate is that the noise must
   * be less than M/4 where M is the gcd of the moduli, in this case is 1. So
   * needs 1/4 of a tooth of accuracy.
   * return calculateMLECRT(m_Feeder.encoderC().getPosition(),
   * m_Feeder.encoderA().getPosition());
   * }
   * 
   */
}
