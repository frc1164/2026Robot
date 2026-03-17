// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Shooter extends SubsystemBase {
  private final SparkMax vert;
  private final SparkMaxConfig vertConfig;

  private final AbsoluteEncoder vertEncoder;
  private final AbsoluteEncoderConfig vertEncoderConfig;

  private final TalonFX shootMot;
  private final TalonFXConfiguration shotConfig;

  private final PIDController vertPID, shotPID;

  private final CommandXboxController CONTROLLER;

  private double lastSpeed;

  /** Creates a new Shooter. */
  public Shooter(CommandXboxController controller) {
    //Instantiate and configure the hood
    vert = new SparkMax(54, MotorType.kBrushless);
    vertConfig = new SparkMaxConfig();

    vertEncoder = vert.getAbsoluteEncoder();
    vertEncoderConfig = new AbsoluteEncoderConfig();
    vertEncoderConfig.zeroOffset(0.1004375 + 0.75)
                     .inverted(false);           

    vertConfig.apply(vertEncoderConfig);               
    vertConfig.inverted(true).idleMode(IdleMode.kBrake); 
    // vertConfig.softLimit.reverseSoftLimit(.006).forwardSoftLimit(.061); //2.16 degrees and 21.96 degrees
    vert.configure(vertConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    vertPID = new PIDController(0.039, 0.00006, 0.0001);//TUNE TUNE TUNE TUNE TUNE before it runs.

    shootMot = new TalonFX(55);
    shotConfig = new TalonFXConfiguration();
    shotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootMot.getConfigurator().apply(shotConfig);

    // shotPID = new PIDController(.001, 0.000025, 0);
    // shotPID = new PIDController(.000000001, 0.00005, 0);
    shotPID = new PIDController(.0001, 0, 0.00003);
    lastSpeed = 0;

    CONTROLLER = controller;
  }

  public double getPhiPosition(){
    return vertEncoder.getPosition() * 360;
  }

  public void runPhiPID(double degrees){
    double angle = -(degrees - 85.6) + 90;

    double power = vertPID.calculate(getPhiPosition(), 105) + (angle-90) * 0.00456368213471;

    if (power > 0.25){
      power = 0.21;
    } else if (power < -0.15){
      power = - 0.17;
    }
    vert.set(power);
    SmartDashboard.putNumber("setpt", angle);
    SmartDashboard.putNumber("location", vertEncoder.getPosition() * 360);
  }

  public void setShotSpeed(double speed){
    // if(shootMot.getVelocity().getValueAsDouble() * 60 < speed){
    //   shootMot.set(1);
    // }else{
    //   shootMot.set(0);
    // }
    double PIDoutput = shotPID.calculate(shootMot.getVelocity().getValueAsDouble() * 60, speed);
    double power = PIDoutput + lastSpeed;
    if (power <= 0 || !DriverStation.isTeleopEnabled()){
      power = 0;
    }
    shootMot.set(power);
    lastSpeed = power;
    SmartDashboard.putNumber("lastSpeed", lastSpeed);
  }

  @Override
  public void periodic() {
    double input = -CONTROLLER.getRightTriggerAxis() * 27 + 83;
    runPhiPID(input);

    SmartDashboard.putNumber("VoltageOut", vert.get());
    SmartDashboard.putNumber("temp", vert.getMotorTemperature());

    SmartDashboard.putNumber("ShotSpeed", shootMot.getVelocity().getValueAsDouble() * 60);
    setShotSpeed(2500);

    //85.6 is hi --> 0 
    //61.4 is low --> 24.2
  }
}
