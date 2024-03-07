// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

   private CANSparkMax m_intakeBottom = new CANSparkMax(Constants.IntakeMotors.m_bottomMotor, CANSparkLowLevel.MotorType.kBrushless);
   private CANSparkMax m_intakeTop = new CANSparkMax(Constants.IntakeMotors.m_topMotor, CANSparkLowLevel.MotorType.kBrushless);
   private CANSparkMax m_intakeTransfer = new CANSparkMax(Constants.IntakeMotors.m_transferMotor, CANSparkLowLevel.MotorType.kBrushless);
  
   public IntakeSubsystem() {}

   public void IntakeNote() {
    m_intakeTop.set(0.4); 
    m_intakeBottom.set(0.4);
  }


  public void stopMotors(){
    m_intakeTop.set(0);
    m_intakeBottom.set(0);
  }

  public void stopTransfer(){
    m_intakeTransfer.set(0);
  }

  public void ShootNote() {
    m_intakeTop.set(-1); 
    m_intakeBottom.set(-1);
    
    new Thread(()->{
      try{
        Thread.sleep(500);
        m_intakeTransfer.set(-1); 
      }catch (Exception e){
      }}).start();

  }

   public void ShootNoteAmp() {
    m_intakeTop.set(-0.3); 
    m_intakeBottom.set(-0.3);

     new Thread(()->{
      try{
        Thread.sleep(200);
        m_intakeTransfer.set(-0.2); 
      }catch (Exception e){
      }}).start();

  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
