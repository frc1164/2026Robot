// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Shooter.AimCommand;
import frc.robot.Shooter.AutoShoot;
import frc.robot.Shooter.Feeder;
import frc.robot.Shooter.ManShot;
import frc.robot.Shooter.ManualShoot;
import frc.robot.Shooter.Shooter;
import frc.robot.Agitator.Agitator;
import frc.robot.Intake.Extend;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Pickup;
import frc.robot.Intake.Retract;
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
  
  // @SuppressWarnings("unused")
  // private final LEDs leds = new LEDs();

  @SuppressWarnings("unused")
  private Agitator m_agitate;
  private Intake m_intake = new Intake();
  private static final Compressor m_compressor = new Compressor(3, PneumaticsModuleType.CTREPCM);


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
    
    // shooter.setDefaultCommand(new AimCommand(shooter, swerve));

    // //this SHOULD be overwritten by auton during auton period I hope, if not then this gets problematic
    // feeder.setDefaultCommand(new AutoShoot(fe   eder, swerve, shooter, agitator));
    m_intake.setDefaultCommand(new Pickup(m_intake, operatorController));
    

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("ShootOn", new ParallelCommandGroup((
        new InstantCommand(() -> feeder.shootOn())),
        new InstantCommand(() -> shooter.setShotSpeed(4000, false)),
        new InstantCommand(() -> agitator.spin())));

    NamedCommands.registerCommand("ShootOff", new ParallelCommandGroup((
        new InstantCommand(() -> feeder.shootOff())),
        new InstantCommand(() -> shooter.setShotSpeed(0, true)),
        new InstantCommand(() -> agitator.stop())));

    NamedCommands.registerCommand("PickupOn", new InstantCommand(() -> m_intake.runPickup(1)));
    NamedCommands.registerCommand("PickupOff", new InstantCommand(() -> m_intake.runPickup(0)));

    NamedCommands.registerCommand("Deploy Intake", new InstantCommand(() -> m_intake.extend()));

    NamedCommands.registerCommand(null, getAutonomousCommand());


    configureBindings();
    m_compressor.enableDigital();
    new Extend(m_intake);
  }

  private void configureBindings() {
    driveController.povDown().onTrue(new InstantCommand(() -> swerve.zeroHeading()));


    // operatorController.y().toggleOnTrue(new ManualShoot(feeder, operatorController, shooter, agitator));
    // operatorController.a().onTrue(new InstantCommand(() -> m_intake.toggleIntake()));
    operatorController.rightTrigger(.25).whileTrue(new ManShot(feeder, shooter, agitator));
    operatorController.povUp().onTrue(new Extend(m_intake));
    operatorController.povDown().onTrue(new Retract(shooter, m_intake));
    // operatorController.rightBumper().whileTrue(new Pickup(m_intake));
  }
 
  

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  public static double getPressure(){
    return m_compressor.getPressure();


  }

}
