// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private CANSparkMax m_leftArmMotor = new CANSparkMax(Constants.ArmMotors.m_armMotorLeft, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax m_rightArmMotor = new CANSparkMax(Constants.ArmMotors.m_armMotorLeft, CANSparkLowLevel.MotorType.kBrushless);

   private SparkPIDController m_LeftPidController;
  
    // CANMotorController.addFollower


  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double rotations;

  public ArmSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
