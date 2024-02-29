// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */

  DoubleSolenoid m_solenoid;

  public void solenoidOpen() {
    m_solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void solenoidClose() {
    m_solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void solenoidOff(){
    m_solenoid.set(DoubleSolenoid.Value.kOff);
  }

  public ClimbSubsystem() {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
     Constants.Pneumatics.kSolenoidForwardChannel,
     Constants.Pneumatics.kSolenoidReverseChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
