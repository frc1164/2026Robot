// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Agitator;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {
  /** Creates a new Agitator. */
  private final SparkMax m_agitate;
  private final SparkMaxConfig agitatorConfig;

  public Agitator() {
    m_agitate = new SparkMax(61, MotorType.kBrushless);
    agitatorConfig = new SparkMaxConfig();
    agitatorConfig.inverted(false)
        .idleMode(IdleMode.kCoast);
    agitatorConfig.smartCurrentLimit(70);
    m_agitate.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void spin() {
    m_agitate.set(.6);
  }

  public void stop() {
    m_agitate.set(0);
  }

  public void reverse(){
    m_agitate.set(-.6);
  }

  public void agitationNation(){
    if (Math.abs(m_agitate.getEncoder().getVelocity()) < 15) {
      reverse();
    } else {
      spin();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
