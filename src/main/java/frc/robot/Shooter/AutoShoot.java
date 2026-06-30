// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Agitator.Agitator;

public class AutoShoot extends Command {
  private final Feeder feeder;
  private final Shooter shooter;
  private final Agitator agitator;

  public AutoShoot(Feeder Feeder, Shooter Shooter, Agitator Agitate) {
    feeder = Feeder;
    shooter = Shooter;
    agitator = Agitate;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
          shooter.shooterGoShoot(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (shooter.aimingAtHub()) {
      
    // }
    // else {
    //   feeder.shootOff();
    //   shooter.shooterGoShoot(false);
    //   agitator.stop();
    // }

    feeder.feedyMcFeedFeed();
      // shooter.shooterGoShoot(true);
      agitator.agitationNation();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      feeder.shootOff();
      shooter.shooterGoShoot(false);
      agitator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
