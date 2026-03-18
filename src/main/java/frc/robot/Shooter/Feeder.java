// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  private final SparkMax feederA, feederB;
  private final SparkMaxConfig feedConfigA, feedConfigB;

  private final AbsoluteEncoder shooterAbsoluteEncoder;
  private final AbsoluteEncoderConfig config1;
  private boolean makeShootGo;

  
  public Feeder() {
    feederA = new SparkMax(56, MotorType.kBrushless);
    feederB = new SparkMax(57, MotorType.kBrushless);
    shooterAbsoluteEncoder = feederB.getAbsoluteEncoder();

    feedConfigA = new SparkMaxConfig();
    feedConfigB = new SparkMaxConfig();
    config1 = new AbsoluteEncoderConfig();

    
    feedConfigA.idleMode(IdleMode.kBrake).inverted(false);
    feedConfigB.idleMode(IdleMode.kBrake).inverted(true).follow(56);

    config1.inverted(false)
           .zeroOffset(0) //subject to change
           .positionConversionFactor(1); //also subject to change but since it is right on the gear probably just 1 or delete the line

   
    feedConfigA.apply(config1);

    feederA.configure(feedConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederB.configure(feedConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
    makeShootGo = false;
  }

  public void shootOn(){
    feederA.set(.5);
  }

  public void shootOff(){
    feederA.set(0);
  }

  public void shootHeld(double speed){
    feederA.set(speed);
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

  //might need to switch these depending on which is wired to which, these are actually the pivot encoders for the shooter though.
  public AbsoluteEncoder getAbsoluteEncoder(){    
    return shooterAbsoluteEncoder;
  }

  @Override
  public void periodic() {}
}
