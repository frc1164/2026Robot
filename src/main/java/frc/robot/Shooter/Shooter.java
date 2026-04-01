// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Shooter extends SubsystemBase {
  private final SparkMax turn;
  private final SparkMaxConfig turnConfig;

  private final RelativeEncoder encoder1;

  private final PIDController thetaPID;

  private double angle;

  private CommandXboxController controller;

  /** Creates a new Shooter. */
  public Shooter(CommandXboxController m_controller) {

    // Instantiate and configure the pivot
    turn = new SparkMax(51, MotorType.kBrushless);
    turnConfig = new SparkMaxConfig();
    turnConfig.inverted(false);
    turnConfig.idleMode(IdleMode.kBrake);

    turnConfig.encoder.velocityConversionFactor(1.0 / 9.0 * 36.0 / 132.0)
        .positionConversionFactor(1.0 / 9.0 * 36.0 / 132.0);

    turn.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder1 = turn.getEncoder();

    controller = m_controller;

    // Instantiate PID's
    thetaPID = new PIDController(0.0125, 0, 0);
    thetaPID.disableContinuousInput();
    angle = .25;
    encoder1.setPosition(angle);
  }

  public double getThetaPosition() {

    angle = encoder1.getPosition() * 360;
    return angle;
  }

  public void runThetaPID(double degrees) {

    degrees = ((degrees % 360) + 360) % 360;

    double pidMotorSpeed = thetaPID.calculate(getThetaPosition(), degrees);
    pidMotorSpeed = Math.max(Math.min(pidMotorSpeed, .75), -.75);
    turn.set(pidMotorSpeed);
    SmartDashboard.putNumber("MotorOutput", pidMotorSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position", getThetaPosition());
    SmartDashboard.putNumber("Target", 180 - controller.getRightX() * 190);
    runThetaPID(180 - controller.getRightX() * 190);
  }
}
