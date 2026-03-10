// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkMax vert;
  private final SparkMaxConfig vertConfig;

  private final SparkMax turn;
  private final SparkMaxConfig turnConfig;
  
  private final CANcoder canCoder1;
  private final CANcoderConfiguration canCoderConfiguration1;
  
  private final CANcoder canCoder2;
  private final CANcoderConfiguration canCoderConfiguration2;

  private final AbsoluteEncoder vertEncoder;
  private final AbsoluteEncoderConfig vertEncoderConfig;

  private final PIDController thetaPID, vertPID;


  private final double gear0TeethCount = 132;
  private final double gear1TeethCount = 17;
  private final double gear2TeethCount = 36;

  private final double n1 = 17; // g1 * n1 (mod g2) = 1
  private final double n2 = 9; // g2 * n2 (mod g1) = 1
  private final double lcm = 612; // lcm(g1, g2)
  

  /** Creates a new Shooter. */
  public Shooter() {

    vert = new SparkMax(54, MotorType.kBrushless);
    turn = new SparkMax(51, MotorType.kBrushless);

    vertConfig = new SparkMaxConfig();
    turnConfig = new SparkMaxConfig();

    // Add Encoder Ids later
    canCoder1 = new CANcoder(52);
    canCoderConfiguration1 = new CANcoderConfiguration();

    canCoder2 = new CANcoder(53);
    canCoderConfiguration2 = new CANcoderConfiguration();

    //Vert needs soft limits, this configurator can apply them, will have them once we know gear ratio
    vertConfig.inverted(false).idleMode(IdleMode.kBrake);
    vert.configure(vertConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnConfig.inverted(false);
    turnConfig.idleMode(IdleMode.kBrake);
    // turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // turnConfig.Feedback.FeedbackRemoteSensorID = 52;
    // turnConfig.Feedback.RotorToSensorRatio = 18.75; //Should find this number
    turn.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    canCoderConfiguration1.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfiguration1.MagnetSensor.MagnetOffset = -0.153320+0.05542;
    canCoder1.getConfigurator().apply(canCoderConfiguration1);

    canCoderConfiguration2.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoder2.getConfigurator().apply(canCoderConfiguration2);

    vertEncoder = vert.getAbsoluteEncoder();
    vertEncoderConfig = new AbsoluteEncoderConfig();
    vertEncoderConfig.positionConversionFactor(0)  //whatever the hood gear ratio is
                     .velocityConversionFactor(0)  //whatever the gear ratio is over 60
                     .zeroOffset(0)                //find this
                     .inverted(false);           //inverted?
    
    
    thetaPID = new PIDController(0.35 * 2 , 0, 0.001);
    vertPID = new PIDController(1, 0, 0);//TUNE TUNE TUNE TUNE TUNE before it runs.
  }

  private double getGear3Rotation(double r1, double r2) {
    final double d1 = r1 * 360;
    final double d2 = r2 * 360;

    final double t1 = d1 * gear1TeethCount / 360;
    final double t2 = d2 * gear2TeethCount / 360;

    final double bezout = (t1 * gear2TeethCount * n2 + t2 * gear1TeethCount * n1) % lcm;

    final double totalRot1 = Math.floor(bezout / gear1TeethCount);

    final double rot0 = (totalRot1 + d1 / 360) * gear1TeethCount / gear0TeethCount * 360;

    return rot0;
  }

  public double getThetaPosition() {
    double gear1Rotation = canCoder1.getPosition().getValueAsDouble() * 360;
    double gear2Rotation = canCoder2.getPosition().getValueAsDouble() * 360;

    return (getGear3Rotation(gear1Rotation, gear2Rotation) % 360) * Math.PI / 180;
    // return canCoder1.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  public double getPhiPosition(){
    return vertEncoder.getPosition() * 2 * Math.PI;
  }

  public void runPhiPID(double radians){
    //might want to set limits here if the motor config soft limits dont work. Or do both.
    vert.set(vertPID.calculate(getPhiPosition(), radians));
  }

  //Once we know the range of theta, we will have to program in limits to this in a weird way, hopefulle we can leave it swapping at 0.
  public void runThetaPID(double radians){//feed this radians
    // double gear1Rotation = canCoder1.getPosition().getValueAsDouble() * 360;
    // double gear2Rotation = canCoder2.getPosition().getValueAsDouble() * 360;

    //PID will not stop running, only recieves updated angles
    

    // double pidGear0Speed = pid.calculate(getTurningPosition(), angle * Math.PI / 180);
    // double pidMotorSpeed = pidGear0Speed * gear0TeethCount / gear1TeethCount;
    double pidMotorSpeed = thetaPID.calculate(getThetaPosition(), radians);
    turn.set(pidMotorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
