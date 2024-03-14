// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.TrapezoidConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.TrapezoidConstants.kP_Translation;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModulePosition.Swerve_Module_Position;
import frc.robot.utils.*;
import frc.robot.utils.PidSlot.PID_SLOT;







public class SwerveModule extends SubsystemBase {

  private final Swerve_Module_Position m_modulePosition;

  private final int m_moduleNumber;

  CANSparkMax m_turnMotor; 
  CANSparkMax m_driveMotor;

  private final SparkPIDController m_driveController;
  private final SparkPIDController m_turnController;
  public final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;

  private final boolean m_CANcoderReversed;
  CANcoder m_CANCoder;
  double m_angleOffset;
  
  double m_currentAngle;
  double m_lastAngle;
  boolean m_initSuccess = false;
  Pose2d m_pose;
  SwerveModuleState m_desiredState = new SwerveModuleState();

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    m_angleOffset, kMaxSpeedMetersPerSecond);

  public SwerveModule(
    Swerve_Module_Position modulePosition,
      CANSparkMax turnMotor,
      CANSparkMax driveMotor,
      CANcoder angleEncoder, 
      boolean turnMotorReversed,
      boolean driveMotorReversed,
      double angleOffset) {
    m_modulePosition = modulePosition;
    m_moduleNumber = m_modulePosition.ordinal();
m_turnMotor = turnMotor;
m_driveMotor = driveMotor;
m_CANCoder = angleEncoder;
m_angleOffset = angleOffset;
m_CANcoderReversed = false;

PidSlot.setDriveMotorConfig(m_driveMotor);
m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

PidSlot.setTurnMotorConfig(m_turnMotor);
m_turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

m_driveEncoder = m_driveMotor.getEncoder();
m_driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveMetersPerEncRev); 
m_driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncRPMperMPS); 

m_turnEncoder = m_turnMotor.getEncoder();
m_turnEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningDegreesPerEncRev);
m_turnEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncRPMtoDPS); 

m_driveController = m_driveMotor.getPIDController();
m_turnController = m_turnMotor.getPIDController(); 

m_turnController.setP(kP_Translation);  
m_turnController.setI(0);   
m_turnController.setD(0);  


m_driveMotor.setInverted(driveMotorReversed);
m_turnMotor.setInverted(turnMotorReversed);


m_lastAngle = getHeadingDegrees();

resetEncoders();

}



  public boolean getInitSuccess() {
        return m_initSuccess;
      }

      public Swerve_Module_Position getModulePosition() {
        return m_modulePosition;
      }

      public int getModuleNumber() {
        return m_moduleNumber;
      }


      public double getAbsoluteEncoderDeg() {

       double angle = m_CANCoder.getAbsolutePosition().getValueAsDouble(); 
       angle *= 360.0;
       return angle * (m_CANcoderReversed ? -1.0 : 1.0);
      }


      public void resetEncoders(){
        m_driveEncoder.setPosition(0);
        m_turnEncoder.setPosition(getAbsoluteEncoderDeg());
        m_turnController.setReference(0, CANSparkMax.ControlType.kPosition); 
      }


        public void resetAngleToAbsolute() {
        double angle = getHeadingDegrees() - m_angleOffset;
        m_turnEncoder.setPosition(angle);
      }
    
    
      public double getHeadingDegrees() {
        if(RobotBase.isReal())
          return m_turnEncoder.getPosition();
        else
          return m_currentAngle;
      }
    

      public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
      }
    

      public double getVelocityMetersPerSecond() {
        return m_driveEncoder.getVelocity();
      }


      public double getTurningVelocity(){
        return m_turnEncoder.getVelocity();
      }


      public double getTurningPosition(){
        return m_turnEncoder.getPosition();
      }
    

      public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001){
          stop();
          return;
        }

         desiredState = PidSlot.optimize(desiredState, getHeadingRotation2d());
        m_driveMotor.set(desiredState.speedMetersPerSecond / kMaxSpeedMetersPerSecond);

            double angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxSpeedMetersPerSecond * 0.01))
                    ? m_lastAngle
                    : desiredState.angle.getDegrees(); 

     m_turnController.setReference(angle, CANSparkMax.ControlType.kPosition, PID_SLOT.POS_SLOT.ordinal());

      }
    

      public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSecond(), getHeadingRotation2d());
      }
    
      public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(m_turnEncoder.getPosition()));
      }
    
      public void setModulePose(Pose2d pose) {
        m_pose = pose;
      }
    
      public Pose2d getModulePose() {
        return m_pose;
      }
    
      public void setDriveNeutralMode(IdleMode m_mode) {
        m_driveMotor.setIdleMode(m_mode);
      }
    
      public void setTurnNeutralMode(IdleMode m_mode) {
        m_turnMotor.setIdleMode(m_mode);
      }
      
      public void stop(){
        m_driveMotor.set(0);
        m_turnMotor.set(0);
      }
    
      //pathplanner testing

      
      private void updateSmartDashboard() {
        SmartDashboard.putNumber("Module " + m_moduleNumber + " Internal Angle",getHeadingDegrees());
        SmartDashboard.putNumber("Module " + m_moduleNumber + " Distance",getPosition().distanceMeters);
        SmartDashboard.putNumber("Module " + m_moduleNumber + " Desired Speed", m_desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Module " + m_moduleNumber + " Speed", getState().speedMetersPerSecond);
      }
    
      @Override
      public void periodic() {
        updateSmartDashboard();
      }
}

