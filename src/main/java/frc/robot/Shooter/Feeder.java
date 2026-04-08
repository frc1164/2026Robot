// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  private final SparkMax feederB, feederA;
  private final SparkMaxConfig feedConfigB, feedConfigA;

  // private final AbsoluteEncoder shooterAbsoluteEncoder;
  // private final AbsoluteEncoderConfig config1;

  public Feeder() {
    feederA = new SparkMax(56, MotorType.kBrushless);
    feederB = new SparkMax(57, MotorType.kBrushless);

    feedConfigA = new SparkMaxConfig();
    feedConfigB = new SparkMaxConfig();

    feedConfigA.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(60);

    feedConfigB.idleMode(IdleMode.kBrake).inverted(true).follow(56).smartCurrentLimit(5);

    feederA.configure(feedConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederB.configure(feedConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void shootOn() {
    feederA.set(1);
  }

  public void shootOff() {
    feederA.set(0);
  }

  public void reverse() {
    feederA.set(-.5);
  }

  public void feedyMcFeedFeed() {
    if (Math.abs(feederA.getEncoder().getVelocity()) < 15) {
      reverse();
    } else {
      shootOn();
    }
  }

  // might need to switch these depending on which is wired to which, these are
  // actually the pivot encoders for the shooter though.
  // public AbsoluteEncoder getAbsoluteEncoder(){
  // return shooterAbsoluteEncoder;
  // }

  @Override
  public void periodic() {
  }
}
