// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  //private final TalonFX m_climbMotor = new TalonFX(Constants.ClimbMotor.m_climbMotor1);

  private CANSparkMax m_climbMotor1 = new CANSparkMax(Constants.ClimbMotor.m_climbMotor1, CANSparkLowLevel.MotorType.kBrushed);
  private CANSparkMax m_climbMotor2 = new CANSparkMax(Constants.ClimbMotor.m_climbMotor2, CANSparkLowLevel.MotorType.kBrushed);


   public ClimbSubsystem() {
   m_climbMotor2.follow(m_climbMotor1);
   //m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void ClimbExtend(){
    m_climbMotor1.set(1);
  }

  public void ClimbRetract(){
    m_climbMotor1.set(-1);
  }

  public void ClimbStop(){
    m_climbMotor1.set(0);
  }

  /*public void ZeroClimber(){
    m_climbMotor1.setPosition(0);
  }*/



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
