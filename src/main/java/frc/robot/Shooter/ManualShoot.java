// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Agitator.Agitator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualShoot extends Command {
  /** Creates a new ManShot. */
  Feeder feeder;
  Shooter shooter;
  Agitator agitator;
  int topLim, bottomLim;

  public ManualShoot(Feeder m_feeder, Shooter m_shooter, Agitator m_agitator) {
    // Use addRequirements() here to declare subsystem dependencies.
    feeder = m_feeder;
    shooter = m_shooter;
    agitator = m_agitator;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.shooterGoShoot(true);
    if (shooter.aimingAtHub()){
      topLim = 3300;
      bottomLim = 3200;
    } else {
      topLim = 6000;
      bottomLim = 3000;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.shooterSpeed() > bottomLim && shooter.shooterSpeed() < topLim && !shooter.aimingAtSelf()){
      feeder.feedyMcFeedFeed();
    } else { 
      feeder.shootOff();
    }
    agitator.agitationNation();
    // feeder.shootOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shooterGoShoot(false);
    agitator.stop();
    feeder.shootOff();
    shooter.resetLastSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
