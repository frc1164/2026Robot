// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterCalculator;
import frc.robot.Shooter.ShooterConstants;

public class LEDs extends SubsystemBase {
  private static final int kPort = 0;
  private static final int kLength = 68; //length NOT updated

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  /** Creates a new LEDs. */
  LEDPattern orange;
  LEDPattern purple;
  LEDPattern red;
  LEDPattern green;
  LEDPattern yellow;

  AddressableLEDBufferView ziaCenter;
  AddressableLEDBufferView ziaArms;
  AddressableLEDBufferView hubStatus;

  public LEDs() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    ziaCenter = m_buffer.createView(56, 67);
    ziaArms = m_buffer.createView(30, 55);
    hubStatus = m_buffer.createView(0, 29);

    //solid color patterns which run at altered brightness 
    orange = LEDPattern.solid(Color.kOrangeRed).atBrightness(Percent.of(15));
    purple = LEDPattern.solid(Color.kPurple).atBrightness(Percent.of(15));
    red = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(25));
    green = LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(25));
    yellow = LEDPattern.solid(Color.kYellow).atBrightness(Percent.of(25));


    runHubStatus(red);
}

//applying colors to zia symbol
 public void runZia() {
    orange.applyTo(ziaCenter);
    purple.applyTo(ziaArms);
    
  }

//applying colors to hub status lights based on hubstate
  public void runHubStatus(LEDPattern pattern) {
    pattern.applyTo(hubStatus);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ShooterConstants.HUBSTATE m_hubstate = ShooterCalculator.isHubActive();

    switch(m_hubstate){
      case INACTIVE: runHubStatus(red);
      break;
      case ACTIVE: runHubStatus(green);
      break;
      case SOON: runHubStatus(yellow);
      break;

    }
    
    runZia();


    m_led.setData(m_buffer);

    SmartDashboard.putString("hubstate", m_hubstate.name());

    //yellow when soon to change
    //green when scoring time
    //red when opposing score
  }
}

