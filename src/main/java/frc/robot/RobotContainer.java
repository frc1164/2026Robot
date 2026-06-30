// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Shooter.AimCommand;
import frc.robot.Shooter.AutoShoot;
import frc.robot.Shooter.Feeder;
import frc.robot.Shooter.ManualShoot;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.UnJamTheShooterLikeABoss;
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
  
  @SuppressWarnings("unused")
  private final LEDs leds = new LEDs();

  private Intake m_intake = new Intake();
  private static final Compressor m_compressor = new Compressor(3, PneumaticsModuleType.CTREPCM);
  private final AutoShoot autonomousShoot;
  private final Pickup autonomousPickup;


  public RobotContainer() {
    swerve = new SwerveSubsystem();
    feeder = new Feeder();
    shooter = new Shooter(swerve, feeder);
    agitator = new Agitator();

    autonomousShoot = new AutoShoot(feeder, shooter, agitator);

    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    autonomousPickup = new Pickup(m_intake, driveController);


    swerve.setDefaultCommand(new SwerveJoystickCmd(
      swerve,
      () -> driveController.getLeftY(),
      () -> driveController.getLeftX(),
      () -> -driveController.getRightX(),
      () -> !driveController.povUp().getAsBoolean()));
    
    //Commands to run constantly unless overwritten
    shooter.setDefaultCommand(new AimCommand(shooter, swerve));
    m_intake.setDefaultCommand(new Extend(m_intake, shooter));
    

    //Commands to use in autonomous routine
    NamedCommands.registerCommand("ShootOn", new InstantCommand(() -> CommandScheduler.getInstance().schedule(autonomousShoot)));
    NamedCommands.registerCommand("ShootOff", new InstantCommand(() -> CommandScheduler.getInstance().cancel(autonomousShoot)));

    NamedCommands.registerCommand("PickupOn", new InstantCommand(() -> CommandScheduler.getInstance().schedule(autonomousPickup)));
    NamedCommands.registerCommand("PickupOff", new InstantCommand(() -> CommandScheduler.getInstance().cancel(autonomousPickup)));

    NamedCommands.registerCommand("Deploy Intake", new Extend(m_intake, shooter));

    autoChooser = AutoBuilder.buildAutoChooser();
    NamedCommands.registerCommand(null, getAutonomousCommand());

    SmartDashboard.putData("Auto Chooser", autoChooser);

    //Apply config for real-time controller inputs
    configureBindings();

    //Turn on compressor for pneomatics
    m_compressor.enableDigital();
  }

  private void configureBindings() {
    //Reset Gyro Yaw to Zero
    driveController.povDown().onTrue(new InstantCommand(() -> swerve.zeroHeading()));

    //Feed to the turret can jam, the automatic unjam is imperfect so this is backup
    operatorController.leftTrigger(.25).whileTrue(new UnJamTheShooterLikeABoss(shooter, agitator, feeder));

    //Shooting Command during manual driving, allows for more nuance than only shooting to score
    operatorController.rightTrigger(.25).whileTrue(new ManualShoot(feeder, shooter, agitator));

    //When this is toggled off, the Default Command triggers and Extends again.
    operatorController.povDown().toggleOnTrue(new Retract(shooter, m_intake));

    //Spins intake roller
    operatorController.rightBumper().whileTrue(new Pickup(m_intake, operatorController));
  }
 
  

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  public static double getPressure(){
    return m_compressor.getPressure();
  }

}
