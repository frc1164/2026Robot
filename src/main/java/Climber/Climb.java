// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
    // DoubleSolenoid corresponds to a double solenoid.
  // In this case, it's connected to channels 1 and 2 of a PH with the default CAN ID.
  private final DoubleSolenoid m_doubleSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
  
      public Climb() {}

      public void extend() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      }
      public void retract(){
        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
