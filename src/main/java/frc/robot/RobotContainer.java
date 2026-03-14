// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Agitator.Agitator;
import frc.robot.Climber.Climb;
import frc.robot.Climber.RunClimbMotors;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Pickup;
import frc.robot.Intake.ToggleIntake;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


public class RobotContainer {
  private Climb m_climb = new Climb();
  private Agitator m_agitate = new Agitator();
  private Intake m_intake = new Intake();
  private final CommandXboxController operatorController;
  private static final Compressor m_compressor = new Compressor(5, PneumaticsModuleType.CTREPCM);


  public RobotContainer() {
    operatorController = new CommandXboxController(1);

    configureBindings();
    m_compressor.enableDigital();
  }

  private void configureBindings() {
    // driveController.x().onTrue(new InstantCommand(()-> m_climb.extend()));
    // driveController.b().onTrue(new InstantCommand(()-> m_climb.retract()));
    // driveController.y().onTrue(new InstantCommand(()-> m_climb.disable()));
    operatorController.povDown().onTrue(new InstantCommand(() -> m_agitate.stop()));
    operatorController.b().onTrue(new ToggleIntake(m_intake, m_climb));
    operatorController.rightBumper().whileTrue(new RunClimbMotors(m_climb, operatorController.getRightY()));
    operatorController.a().whileTrue(new Pickup(m_intake));
  }
 
  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


  public static double getPressure(){
    return m_compressor.getPressure();


  }

}
