// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final TalonFX m_climbMotor = new TalonFX(Constants.ClimbMotor.m_climbMotor1);

  public void ClimbExtend(){
    m_climbMotor.set(0.4);
  }

  public void ClimbRetract(){
    m_climbMotor.set(-0.4);
  }

  public void ClimbStop(){
    m_climbMotor.set(0);
  }

  public void ZeroClimber(){
    m_climbMotor.setPosition(0);
  }



  public ClimbSubsystem() {
   
   m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
