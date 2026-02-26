// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


//ALL DISTANCE MEASUREMENTS IN METERS
//ALL ANGLE MEASUREMENTS IN RADIANS
public class ShooterConstants{
  public static final class SHOOTEROFFSETS{
    public static final double vertical = 0;
    public static final double translation = 0;
    public static final double theta = 0;
  }

  public static final double exitVelocity = 0; 
  public static final Translation3d hubPose = new Translation3d(0,0,0); 
  public static final double targetHeightFromShooter = hubPose.getZ() - SHOOTEROFFSETS.vertical;
  public static final double gravity = 9.81; //We do NOT need any more accurate than this

  public static final InterpolatingDoubleTreeMap shotMap = new InterpolatingDoubleTreeMap();
  static{
    //example data point
    shotMap.put(0.0, Math.PI/2);
    //first is dist, second is radians
  }
}
