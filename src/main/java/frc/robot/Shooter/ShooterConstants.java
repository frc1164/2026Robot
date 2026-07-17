// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.Shooter.ShooterCalculator.ShotInfo;


//ALL DISTANCE MEASUREMENTS IN METERS
public class ShooterConstants{

  //Offset of shooter from bot center
  public static final class SHOOTEROFFSETS{
    public static final double vertical = 0.47625;
    public static final double translation = 0.1709801;
    public static final double theta = 2.11746;
  }

  //Divvies up field by x-coordinate- uused in target selection
  public static final class XVALS{
    public static final double BATRENCH = 4.0;
    public static final double BMTRENCH = 5.4;
    public static final double RMTRENCH = 11.3;
    public static final double RATRENCH = 12.5;
    public static final double REDWALL = 16.5;
  }


  //3D Coordinates of every conditional shot target
  public static final class TAGRETS{
    public static final Translation3d BLUEHUB = new Translation3d(4.63,4.0,1.68);
    public static final Translation3d BLUEPASSUP = new Translation3d(2.5, 6, 0);
    public static final Translation3d BLUEPASSDOWN = new Translation3d(2.5, 2, 0);
    public static final Translation3d REDHUB = new Translation3d(11.919, 4.0, 1.68);
    public static final Translation3d REDPASSUP = new Translation3d(14.5, 6, 0);
    public static final Translation3d REDPASSDOWN = new Translation3d(14.5, 2, 0);
    public static final Translation3d CENTERDOWN = new Translation3d(8.25, 1.5, 0);
    public static final Translation3d CENTERUP = new Translation3d(8.25, 6.345, 0);
  }

  public static enum HUBSTATE{
    SOON, ACTIVE, INACTIVE;
  };
  
  public static final double exitVelocity = 13.2994089; //5000rpm to m/s wheel is 4 in radius. Divide tangential velo by 2 for slippage(tested factor). 
  public static final double minVert = 57; //degrees from horizontal
  public static final double maxVert = 85.6; //degrees from horizontal
  public static final Translation3d hubPose = new Translation3d(4.63,4.0,1.68); //should be unused
  public static final double targetHeightFromShooter = hubPose.getZ() - SHOOTEROFFSETS.vertical;
  public static final double gravity = 9.81; //We do NOT need any more accurate than this


  //Use linear regression to create fit data that can be called at any time
  public static final InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();
  public static final InterpolatingTreeMap<Double, ShotInfo> shotMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotInfo::interpolate);
  static{
    //example data point
      // shotMap.put(0.0, new ShotInfo(exitVelocity, Math.PI/2));
      // timeMap.put(0.0, Math.PI/2);
    //first is dist fired, second is degrees from straight down
    shotMap.put(4.38, new ShotInfo(exitVelocity, 107));
    timeMap.put(3.13, 1.45);

    shotMap.put(3.6871, new ShotInfo(exitVelocity,102));
    timeMap.put(4.15, 1.376);

    shotMap.put(1.9153, new ShotInfo(exitVelocity,99));
    timeMap.put(2.00, 1.436);

    shotMap.put(1.6, new ShotInfo(exitVelocity, 96));
    timeMap.put(1.1, 1.284);
    timeMap.put(4.5, 1.451);

    shotMap.put(2.75, new ShotInfo(exitVelocity, 99.5));
    shotMap.put(3.822, new ShotInfo(exitVelocity,104.5));
    shotMap.put(4.9, new ShotInfo(exitVelocity, 109.5));
  }
}
