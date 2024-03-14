// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private CANSparkMax m_leftArmMotor = new CANSparkMax(Constants.ArmMotors.m_armMotorLeft, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax m_rightArmMotor = new CANSparkMax(Constants.ArmMotors.m_armMotorRight, CANSparkLowLevel.MotorType.kBrushless);

   private SparkPIDController m_LeftPidController;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double rotations;

  public ArmSubsystem() {

    m_rightArmMotor.follow(m_leftArmMotor,true);

    m_LeftPidController = m_leftArmMotor.getPIDController();
    
    
    kP = Constants.ZeroPIDCoefficients.m_ZerokP; 
    kI = Constants.ZeroPIDCoefficients.m_ZerokI;
    kD = Constants.ZeroPIDCoefficients.m_ZerokD; 
    kIz = Constants.ZeroPIDCoefficients.m_ZerokIz; 
    kFF = Constants.ZeroPIDCoefficients.m_ZerokFF; 
    kMaxOutput = Constants.ZeroPIDCoefficients.m_ZerokMaxOutput; 
    kMinOutput = Constants.ZeroPIDCoefficients.m_ZerokMinOutput;

    setAllPIDValues(kP,kI,kD,kIz,kFF,kMinOutput,kMaxOutput);

  
    
    SmartDashboard.putNumber("Arm P Gain", kP);
    SmartDashboard.putNumber("Arm I Gain", kI);
    SmartDashboard.putNumber("Arm D Gain", kD);
    SmartDashboard.putNumber("Arm I Zone", kIz);
    SmartDashboard.putNumber("Arm Feed Forward", kFF);
    SmartDashboard.putNumber("Arm Max Output", kMaxOutput);
    SmartDashboard.putNumber("Arm Min Output", kMinOutput);
    SmartDashboard.putNumber("Arm Set Rotations", 0);
 
    double ArmPValue = m_LeftPidController.getP();

    SmartDashboard.putNumber("Arm P Value", ArmPValue);

  }

  public void setArmReference(double speed, CANSparkMax.ControlType type) {

    this.rotations = speed;
    m_LeftPidController.setReference(speed, type);

  }

  public boolean isAtSetpoint() {
    return Math.abs(m_leftArmMotor.getEncoder().getPosition() - rotations) <= 0.5;
  }
 
  public void setAllPIDValues(double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput){


    m_LeftPidController.setP(kP);
    m_LeftPidController.setI(kI);
    m_LeftPidController.setD(kD);
    m_LeftPidController.setIZone(kIz);
    m_LeftPidController.setFF(kFF);
    m_LeftPidController.setOutputRange(kMinOutput, kMaxOutput);
 
   
  }


  public void setPIDValues(double kP, double kI, double kD, double kMinOutput, double kMaxOutput) {

    m_LeftPidController.setP(kP);
    m_LeftPidController.setOutputRange(kMinOutput, kMaxOutput);
  }

 @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm right",m_rightArmMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("arm left",m_leftArmMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("arm lrft current",m_leftArmMotor.getOutputCurrent());
    SmartDashboard.putBoolean("arm at setpoint", isAtSetpoint());
  }

}
