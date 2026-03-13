// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  // DoubleSolenoid corresponds to a double solenoid.
  // In this case, it's connected to channels 1 and 2 of a PH with the default CANID.
  private final DoubleSolenoid m_extendSolenoid, lockingSolenoid;
  private final DigitalInput lockSensor;

  private final SparkMax climbMotorA, climbMotorB;
  private final SparkMaxConfig climbMotorConfigA, climbMotorConfigB;

  public Climb() {
    m_extendSolenoid = new DoubleSolenoid(5, PneumaticsModuleType.CTREPCM, 2, 5);// CHANGE THESE CHANNELS
    lockingSolenoid = new DoubleSolenoid(5, PneumaticsModuleType.CTREPCM, 2, 5);// CHANGE THESE CHANNELS
    
    lockSensor = new DigitalInput(9);// Maybe change port?? depends on build.
    
    climbMotorA = new SparkMax(61, MotorType.kBrushless);
    climbMotorB = new SparkMax(62, MotorType.kBrushless);

    climbMotorConfigA = new SparkMaxConfig();
    climbMotorConfigA.inverted(true)
                     .idleMode(IdleMode.kBrake);
    climbMotorA.configure(climbMotorConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climbMotorConfigB = new SparkMaxConfig();
    climbMotorConfigB.inverted(false)
                     .idleMode(IdleMode.kBrake);
    climbMotorB.configure(climbMotorConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  
  public boolean isExtended(){
    return lockSensor.get();
  }

  public void extend() {
    m_extendSolenoid.set(DoubleSolenoid.Value.kForward);
    // include sensor to trigger locking solenoid
  }

  public void retract() {
    m_extendSolenoid.set(DoubleSolenoid.Value.kReverse);
    //probably include unlock solenoid here too but that might actuaally have to be a separate method with some timing constraints or something.
  }

  public void disable() {
    m_extendSolenoid.set(DoubleSolenoid.Value.kOff);
  }

  public void runClimber(double speed){
    climbMotorA.set(speed);
    climbMotorB.set(speed);
  }

  public void toggleClimber(){
    if(isExtended()){
      retract();
    }else if (!isExtended()){
      extend();
    }
  }

  //Probably create a PID method based on the chassis pitch?

  @Override
  public void periodic() {

  }
}
