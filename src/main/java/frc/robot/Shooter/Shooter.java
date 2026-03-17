// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkMax turn;
  private final SparkMaxConfig turnConfig;

  private final AbsoluteEncoder encoder1;
  private final AbsoluteEncoder encoder2;

  private final AbsoluteEncoderConfig config1;

  private final PIDController thetaPID;

  private final Feeder feeder;

  private final double gear0TeethCount = 132;
  private final double gear1TeethCount = 17;
  private final double gear2TeethCount = 36;

  private final double n1 = 17; // g1 * n1 (mod g2) = 1
  private final double n2 = 9; // g2 * n2 (mod g1) = 1
  private final double lcm = 612; // lcm(g1, g2)

  /** Creates a new Shooter. */
  public Shooter(Feeder m_feeder) {

    // Instantiate and configure the pivot
    turn = new SparkMax(51, MotorType.kBrushless);
    turnConfig = new SparkMaxConfig();
    turnConfig.inverted(false);
    turnConfig.idleMode(IdleMode.kBrake);

    feeder = m_feeder;
    encoder1 = feeder.getEncoder2(); // all the configuration logic occurs in Feeder
    encoder2 = turn.getAbsoluteEncoder(); 
    config1 = new AbsoluteEncoderConfig();
    config1.inverted(false)
           .zeroOffset(0.5370022);; //subject to change
    turnConfig.apply(config1);    
    turn.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Instantiate PID's
    thetaPID = new PIDController(0.0001, 0, 0);
  }

  private double getGear3Rotation(double r1, double r2) {
    final double d1 = r1 * 360;
    final double d2 = r2 * 360;

    SmartDashboard.putNumber("d1", d1);
    SmartDashboard.putNumber("d2", d2);

    final double t1 = d1 * gear1TeethCount / 360;
    final double t2 = d2 * gear2TeethCount / 360;

    SmartDashboard.putNumber("t1", t1);
    SmartDashboard.putNumber("t2", t2);

    final double bezout = (t1 * gear2TeethCount * n2 + t2 * gear1TeethCount * n1) % lcm;

    SmartDashboard.putNumber("bezout", bezout);

    final double totalRot1 = Math.floor(bezout / gear1TeethCount);

    SmartDashboard.putNumber("totalrot1", totalRot1);

    final double rot0 = (totalRot1 + d1 / 360) * gear1TeethCount / gear0TeethCount * 360;

    SmartDashboard.putNumber("rot0 - maingear", rot0);

    return totalRot1;
  }

  public double getThetaPosition() {
    double gear1Rotation = encoder1.getPosition();
    double gear2Rotation = encoder2.getPosition();

    return (getGear3Rotation(gear1Rotation, gear2Rotation) % 360) * Math.PI / 180;
  }

  // Once we know the range of theta, we will have to program in limits to this in
  // a weird way, hopefulle we can leave it swapping at 0.
  public void runThetaPID(double radians) {
    // PID will not stop running, only recieves updated angles

    double pidGear0Speed = thetaPID.calculate(getThetaPosition(), radians * Math.PI / 180);
    double pidMotorSpeed = pidGear0Speed * gear0TeethCount / gear1TeethCount;
    // double pidMotorSpeed = thetaPID.calculate(getThetaPosition(), radians);
    turn.set(pidMotorSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("main gear", getThetaPosition());
    SmartDashboard.putNumber("gear1", encoder1.getPosition());
    SmartDashboard.putNumber("gear 2", encoder2.getPosition());
  }
}
