// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */

  CANSparkMax m_wristMotorLeft = new CANSparkMax(Constants.WristMotors.m_wristMotorLeft, CANSparkLowLevel.MotorType.kBrushed);
  CANSparkMax m_wristMotorRight = new CANSparkMax(Constants.WristMotors.m_wristMotorRight, CANSparkLowLevel.MotorType.kBrushed);

  //WPI_TalonSRX m_wristMotorLeft = new WPI_TalonSRX(Constants.WristMotors.m_wristMotorLeft);
  //WPI_TalonSRX m_wristMotorRight = new WPI_TalonSRX(Constants.WristMotors.m_wristMotorRight);

  private SparkPIDController m_WristPidController;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double rotations;

  public WristSubsystem() {
   m_wristMotorRight.follow(m_wristMotorLeft);

    m_WristPidController = m_wristMotorLeft.getPIDController();
    
    
    kP = Constants.ZeroPIDCoefficients.m_ZerokP; 
    kI = Constants.ZeroPIDCoefficients.m_ZerokI;
    kD = Constants.ZeroPIDCoefficients.m_ZerokD; 
    kIz = Constants.ZeroPIDCoefficients.m_ZerokIz; 
    kFF = Constants.ZeroPIDCoefficients.m_ZerokFF; 
    kMaxOutput = Constants.ZeroPIDCoefficients.m_ZerokMaxOutput; 
    kMinOutput = Constants.ZeroPIDCoefficients.m_ZerokMinOutput;

    setAllPIDValues(kP,kI,kD,kIz,kFF,kMinOutput,kMaxOutput);

  
    
    SmartDashboard.putNumber("Elbow P Gain", kP);
    SmartDashboard.putNumber("Elbow I Gain", kI);
    SmartDashboard.putNumber("Elbow D Gain", kD);
    SmartDashboard.putNumber("Elbow I Zone", kIz);
    SmartDashboard.putNumber("Elbow Feed Forward", kFF);
    SmartDashboard.putNumber("Elbow Max Output", kMaxOutput);
    SmartDashboard.putNumber("Elbow Min Output", kMinOutput);
    SmartDashboard.putNumber("Elbow Set Rotations", 0);
 
    double ElbowPValue = m_WristPidController.getP();
    SmartDashboard.putNumber("Elbow P Value", ElbowPValue);

  }

  public void setArmReference(double speed, CANSparkMax.ControlType type) {

    this.rotations = speed;
    m_WristPidController.setReference(speed, type);

  }

  public boolean isAtSetpoint() {
    return Math.abs(m_wristMotorLeft.getEncoder().getPosition() - rotations) <= 0.5;
  }
 
  public void setAllPIDValues(double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput){


    m_WristPidController.setP(kP);
    m_WristPidController.setI(kI);
    m_WristPidController.setD(kD);
    m_WristPidController.setIZone(kIz);
    m_WristPidController.setFF(kFF);
    m_WristPidController.setOutputRange(kMinOutput, kMaxOutput);
 
   
  }


  public void setPIDValues(double kP, double kI, double kD, double kMinOutput, double kMaxOutput) {

    m_WristPidController.setP(kP);
  }


public void setAutoPIDValues(double kP) {
  
  m_WristPidController.setP(kP);
  m_WristPidController.setI(0.0);
  m_WristPidController.setD(0.0);
  m_WristPidController.setIZone(0.0);
  m_WristPidController.setFF(0.0);
  m_WristPidController.setOutputRange(-0.15, 0.15);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
