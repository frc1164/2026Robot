// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Time;
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

  LEDPattern lastPattern;

  public LEDs() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    // ziaCenter = m_buffer.createView(56, 67);
    // ziaArms = m_buffer.createView(30, 55);
    // hubStatus = m_buffer.createView(0, 29);

    hubStatus = m_buffer.createView(0, 67);

    //solid color patterns which run at altered brightness 
    orange = LEDPattern.solid(Color.kOrangeRed).atBrightness(Percent.of(15));
    purple = LEDPattern.solid(Color.kPurple).atBrightness(Percent.of(15));
    red = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(25));
    green = LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(25));
    yellow = LEDPattern.solid(Color.kYellow).atBrightness(Percent.of(25));

    runHubStatus(red);
    lastPattern = LEDPattern.kOff;
}

//applying colors to zia symbol
//  public void runZia() {
//     orange.applyTo(ziaCenter);
//     purple.applyTo(ziaArms);
    
//   }

  //pulsify
  private LEDPattern pulsify(LEDPattern base){
    return base.breathe(Time.ofBaseUnits(.5, Second));
  }

  private LEDPattern countUP(){
    LEDPattern mask = LEDPattern.progressMaskLayer(() -> (Robot.getHubTime() - 5) / 20);
    return purple.mask(mask);
  }

  private LEDPattern countDOWN(){
    LEDPattern mask = LEDPattern.progressMaskLayer(() -> (20 - (Robot.getHubTime() - 5) ) / 20);
    return orange.mask(mask);
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
      case INACTIVE: runHubStatus(countDOWN()); lastPattern = purple;
      break;
      case ACTIVE: runHubStatus(countUP()); lastPattern = orange;
      break;
      case SOON: runHubStatus(pulsify(lastPattern));
      break;
    }
    
    // runZia();


    m_led.setData(m_buffer);

    SmartDashboard.putString("hubstate", m_hubstate.name());

    //orange countdown when active till 5 seconds before
    //pulse orange when soon to deactiate
    //purple count up when inactive till 5 seconds before
    //pulse purple when soon to activate

  }
}

