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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Shooter extends SubsystemBase {
  private final SparkMax vert;
  private final SparkMaxConfig vertConfig;

  private final AbsoluteEncoder vertEncoder;
  private final AbsoluteEncoderConfig vertEncoderConfig;

  private final PIDController vertPID;

  private final CommandXboxController CONTROLLER;

  /** Creates a new Shooter. */
  public Shooter(CommandXboxController controller) {
    //Instantiate and configure the hood
    vert = new SparkMax(54, MotorType.kBrushless);
    vertConfig = new SparkMaxConfig();

    vertEncoder = vert.getAbsoluteEncoder();
    vertEncoderConfig = new AbsoluteEncoderConfig();
    vertEncoderConfig.zeroOffset(0.105)
                     .inverted(false);           

    vertConfig.apply(vertEncoderConfig);               
    vertConfig.inverted(true).idleMode(IdleMode.kBrake); 
    vertConfig.softLimit.reverseSoftLimit(.006).forwardSoftLimit(.061); //2.16 degrees and 21.96 degrees
    vert.configure(vertConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    vertPID = new PIDController(0.03, 0.00002, 0.0001);//TUNE TUNE TUNE TUNE TUNE before it runs.
  

    CONTROLLER = controller;
  }

  public double getPhiPosition(){
    return vertEncoder.getPosition() * 360;
  }

  public void runPhiPID(double degrees){
    double angle = -(degrees - 85.6);

    if(5 <= angle || angle <= 25){
      vert.set(vertPID.calculate(getPhiPosition(), angle) + angle * (0.00214));
    }
    SmartDashboard.putNumber("setpt", angle);
    SmartDashboard.putNumber("location", vertEncoder.getPosition() * 360);
  }

  @Override
  public void periodic() {
    double input = -CONTROLLER.getRightTriggerAxis() * 25 + 80;
    runPhiPID(input);

    SmartDashboard.putNumber("VoltageOut", vert.get());
    SmartDashboard.putNumber("temp", vert.getMotorTemperature());


    //85.6 is hi --> 0 
    //61.4 is low --> 24.2
  }
}
