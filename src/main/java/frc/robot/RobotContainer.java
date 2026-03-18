// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Shooter.AimCommand;
import frc.robot.Shooter.AutoShoot;
import frc.robot.Shooter.Feeder;
import frc.robot.Shooter.ManualShoot;
import frc.robot.Shooter.Shooter;
import frc.robot.Agitator.Agitator;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Pickup;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Swerve.SwerveSubsystem;
import frc.robot.Swerve.SwerveJoystickCmd;


public class RobotContainer {
  private final SwerveSubsystem swerve;
  private final Shooter shooter;
  private final Feeder feeder;
  private final Agitator agitator;

  private final CommandXboxController driveController, operatorController;

  private final SendableChooser<Command> autoChooser;
  
  @SuppressWarnings("unused")
  private final LEDs leds = new LEDs();

  @SuppressWarnings("unused")
  private Agitator m_agitate = new Agitator();
  private Intake m_intake = new Intake();
  private static final Compressor m_compressor = new Compressor(5, PneumaticsModuleType.CTREPCM);


  public RobotContainer() {
    swerve = new SwerveSubsystem();
    feeder = new Feeder();
    shooter = new Shooter(feeder);
    agitator = new Agitator();

    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    swerve.setDefaultCommand(new SwerveJoystickCmd(
      swerve,
      () -> driveController.getLeftY(),
      () -> driveController.getLeftX(),
      () -> -driveController.getRightX(),
      () -> !driveController.povUp().getAsBoolean()));
    
    shooter.setDefaultCommand(new AimCommand(shooter, swerve));

    //this SHOULD be overwritten by auton during auton period I hope, if not then this gets problematic
    feeder.setDefaultCommand(new AutoShoot(feeder, swerve, shooter, agitator));
    

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);


    configureBindings();
    m_compressor.enableDigital();
  }

  private void configureBindings() {
    driveController.povDown().onTrue(new InstantCommand(() -> swerve.zeroHeading()));
    operatorController.y().toggleOnTrue(new ManualShoot(feeder, operatorController, shooter, agitator));
    operatorController.a().onTrue(new InstantCommand(() -> m_intake.toggleIntake()));
    operatorController.rightBumper().whileTrue(new Pickup(m_intake));
  }
 
  

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  public static double getPressure(){
    return m_compressor.getPressure();


  }

}
