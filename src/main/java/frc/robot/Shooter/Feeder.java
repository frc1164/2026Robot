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

  private final SparkMax feederA;
  private final SparkMaxConfig feedConfigA;

  private final AbsoluteEncoder encoder2;
  private final AbsoluteEncoderConfig config2;
  private boolean makeShootGo;

  
  public Feeder() {
    feederA = new SparkMax(57, MotorType.kBrushless);
    //feederB = new SparkMax(57, MotorType.kBrushless);
    encoder2 = feederA.getAbsoluteEncoder();
    //encoder1 = feederB.getAbsoluteEncoder();

    feedConfigA = new SparkMaxConfig();
    //feedConfigB = new SparkMaxConfig();
    //config1 = new AbsoluteEncoderConfig();
    config2 = new AbsoluteEncoderConfig();

    
    feedConfigA.idleMode(IdleMode.kBrake).inverted(false);
    //feedConfigB.idleMode(IdleMode.kBrake).inverted(true).follow(56);

    // config1.inverted(false)
    //        .zeroOffset(0.5370022); //subject to change

    config2.inverted(false)
           .zeroOffset(0.5882150); //subject to change
    
    feedConfigA.apply(config2);
    //feedConfigB.apply(config1);


    feederA.configure(feedConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //feederB.configure(feedConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
    //makeShootGo = false;
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

  //might need to switch these depending on which is wired to which, these are actually the pivot encoders for the shooter though.
  public AbsoluteEncoder getEncoder2(){    
    return encoder2;
  }


  @Override
  public void periodic() {}
}
