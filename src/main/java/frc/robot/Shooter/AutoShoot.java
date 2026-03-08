// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Swerve.SwerveSubsystem;

public class AutoShoot extends Command {
  private final Feeder feeder;
  private final SwerveSubsystem swerve;
  private Translation3d HUB, PASSUP, PASSDOWN; 

  public AutoShoot(Feeder Feeder, SwerveSubsystem Swerve) {
    swerve = Swerve;
    feeder = Feeder;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)){
      HUB = ShooterConstants.TAGRETS.BLUEHUB;
      PASSUP = ShooterConstants.TAGRETS.BLUEPASSUP;
      PASSDOWN = ShooterConstants.TAGRETS.BLUEPASSDOWN;
    }
    else if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)){
      HUB = ShooterConstants.TAGRETS.REDHUB;
      PASSUP = ShooterConstants.TAGRETS.REDPASSUP;
      PASSDOWN = ShooterConstants.TAGRETS.REDPASSDOWN; 
    }
    else{
      HUB = ShooterConstants.TAGRETS.BLUEHUB;
      PASSUP = ShooterConstants.TAGRETS.BLUEPASSUP;
      PASSDOWN = ShooterConstants.TAGRETS.BLUEPASSDOWN;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ShooterCalculator.target(swerve.getPose()) == HUB && ShooterCalculator.isHubActive()){
      feeder.shootOn();
    }
    else if (ShooterCalculator.target(swerve.getPose()) == PASSUP || ShooterCalculator.target(swerve.getPose()) == PASSDOWN){
      feeder.shootOn();
    }
    else {feeder.shootOff();}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
