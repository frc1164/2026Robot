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

  private final SparkMax feederA, feederB;
  private final SparkMaxConfig feedConfigA, feedConfigB;
  private boolean makeShootGo;

  
  public Feeder() {
    feederA = new SparkMax(56, MotorType.kBrushless);
    feederB = new SparkMax(57, MotorType.kBrushless);

    feedConfigA = new SparkMaxConfig();
    feedConfigB = new SparkMaxConfig();

    
    feedConfigA.idleMode(IdleMode.kBrake).inverted(false);
    feederA.configure(feedConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    feedConfigB.idleMode(IdleMode.kBrake).inverted(true).follow(56);
    feederB.configure(feedConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    makeShootGo = false;

  }

  public void shootOn(){
    feederA.set(.5);
  }

  public void shootOff(){
    feederA.set(0);
  }

  public void toggleShoot(){
    if(makeShootGo){
      shootOn();
      makeShootGo = !makeShootGo;
    }
    else{
      shootOff();
      makeShootGo = !makeShootGo;
    }
  }

  @Override
  public void periodic() {}
}
