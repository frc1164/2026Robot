// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {

  private final SparkFlex m_pickup;
  private final SparkFlexConfig pickupMotConfig;
  private final DoubleSolenoid m_extendSolenoid;
  public boolean ranit;

  public Intake() {
    m_pickup = new SparkFlex(60, MotorType.kBrushless);
    pickupMotConfig = new SparkFlexConfig();
    pickupMotConfig.inverted(false)
        .idleMode(IdleMode.kCoast);

    m_pickup.configure(pickupMotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_extendSolenoid = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, 5, 2);

    ranit = false;
    // extend(); // starting state
  }

  public void runPickup(double speed) {
    m_pickup.set(speed);
    SmartDashboard.putNumber("intake speed", speed);
  }

  public void extend() {
    m_extendSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    m_extendSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void disable() {
    m_extendSolenoid.set(DoubleSolenoid.Value.kOff);

  }

  public boolean intakeExtended() {
    return m_extendSolenoid.get() == DoubleSolenoid.Value.kForward;
  }

  public void toggleIntake() {
    if (intakeExtended()) {
      retract();
    } else if (!intakeExtended()) {
      extend();
    }
  }

  public void yesItRan(){
    ranit = true;
  }
  public boolean didItRun(){
    return ranit;
  }
  // the climb and intake cannot both be extended at the same time so when you
  // write the command to extend one, hte other needs to first be retracted.
  // the extension and retraction command can probably be a toggle between the two
  // today can you just write the subsystem for the intake

  @Override
  public void periodic() {
  }
}
