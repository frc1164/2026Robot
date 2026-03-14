// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;


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
  
  private final AbsoluteEncoder encoder1;
  private final AbsoluteEncoder encoder2;

  private final AbsoluteEncoder vertEncoder;
  private final AbsoluteEncoderConfig vertEncoderConfig;

  private final PIDController thetaPID, vertPID;

  private final Feeder feeder;

  private final double gear0TeethCount = 132;
  private final double gear1TeethCount = 17;
  private final double gear2TeethCount = 36;

  private final double n1 = 17; // g1 * n1 (mod g2) = 1
  private final double n2 = 9; // g2 * n2 (mod g1) = 1
  private final double lcm = 612; // lcm(g1, g2)
  

  /** Creates a new Shooter. */
  public Shooter(Feeder m_feeder) {


    //Instantiate and configure the pivot
    turn = new SparkMax(51, MotorType.kBrushless);
    turnConfig = new SparkMaxConfig();
    turnConfig.inverted(false);
    turnConfig.idleMode(IdleMode.kBrake);
    turn.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    feeder = m_feeder;
    encoder1 = feeder.getEncoder1(); //all the configuration logic occurs in Feeder
    encoder2 = feeder.getEncoder2(); //all the configuration logic occurs in Feeders


    //Instantiate and configure the hood
    vert = new SparkMax(54, MotorType.kBrushless);
    vertConfig = new SparkMaxConfig();

    vertEncoder = vert.getAbsoluteEncoder();
    vertEncoderConfig = new AbsoluteEncoderConfig();
    vertEncoderConfig.positionConversionFactor(0)  //whatever the hood gear ratio is
                     .velocityConversionFactor(0)  //whatever the gear ratio is over 60
                     .zeroOffset(0)                //find this
                     .inverted(false);           //inverted?

    vertConfig.apply(vertEncoderConfig);               
    vertConfig.inverted(false).idleMode(IdleMode.kBrake);     //Hood needs soft limits, this configurator can apply them, will have them once we know gear ratio
    vert.configure(vertConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //Instantiate PID's
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
    double gear1Rotation = encoder1.getPosition() * 360;
    double gear2Rotation = encoder2.getPosition() * 360;

    return (getGear3Rotation(gear1Rotation, gear2Rotation) % 360) * Math.PI / 180;
  }

  public double getPhiPosition(){
    return vertEncoder.getPosition() * 2 * Math.PI;
  }

  public void runPhiPID(double radians){
    //might want to set limits here if the motor config soft limits dont work. Or do both.
    vert.set(vertPID.calculate(getPhiPosition(), radians));
  }

  //Once we know the range of theta, we will have to program in limits to this in a weird way, hopefulle we can leave it swapping at 0.
  public void runThetaPID(double radians){
    //PID will not stop running, only recieves updated angles 

    // double pidGear0Speed = pid.calculate(getTurningPosition(), angle * Math.PI / 180);
    // double pidMotorSpeed = pidGear0Speed * gear0TeethCount / gear1TeethCount; //this should be handles by position and velocity conversion factors
    double pidMotorSpeed = thetaPID.calculate(getThetaPosition(), radians);
    turn.set(pidMotorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
