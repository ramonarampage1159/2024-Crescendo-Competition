// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.networktables.DoublePublisher;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.IDs;

import frc.robot.Constants.ModulePosition.Swerve_Module_Position;
import frc.robot.utils.ModuleMap;


import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.TimedRobot;

import com.kauailabs.navx.frc.AHRS;

public class SwerveDrivetrain extends SubsystemBase {


 private final HashMap<Swerve_Module_Position, SwerveModule> m_swerveModules =
      new HashMap<>(
          Map.of(
            Swerve_Module_Position.FRONT_LEFT,
                  new SwerveModule(
                    Swerve_Module_Position.FRONT_LEFT,
                      new CANSparkMax(IDs.kFrontLeftSteerMotor, CANSparkLowLevel.MotorType.kBrushless),
                      new CANSparkMax(IDs.kFrontLeftDriveMotor, CANSparkLowLevel.MotorType.kBrushless),
                      new CANcoder(IDs.kFrontLeftCanCoder), 
                      Constants.DriveConstants.kFrontLeftTurningMotorReversed, 
                      Constants.DriveConstants.kFrontLeftDriveMotorReversed,
                      Constants.IDs.kFrontLeftModuleOffset),
            Swerve_Module_Position.FRONT_RIGHT,
                  new SwerveModule(
                    Swerve_Module_Position.FRONT_RIGHT,
                      new CANSparkMax(IDs.kFrontRightSteerMotor, CANSparkLowLevel.MotorType.kBrushless),
                      new CANSparkMax(IDs.kFrontRightDriveMotor, CANSparkLowLevel.MotorType.kBrushless),
                      new CANcoder(IDs.kFrontRightCanCoder),
                      Constants.DriveConstants.kFrontRightTurningMotorReversed,
                      Constants.DriveConstants.kFrontRightDriveMotorReversed,
                      Constants.IDs.kFrontRightModuleOffset),
            Swerve_Module_Position.BACK_LEFT,
                  new SwerveModule(
                    Swerve_Module_Position.BACK_LEFT,
                      new CANSparkMax(IDs.kRearLeftSteerMotor, CANSparkLowLevel.MotorType.kBrushless),
                      new CANSparkMax(IDs.kRearLeftDriveMotor, CANSparkLowLevel.MotorType.kBrushless),
                      new CANcoder(IDs.kRearLeftCanCoder),
                      Constants.DriveConstants.kBackLeftTurningMotorReversed,
                      Constants.DriveConstants.kBackLeftDriveMotorReversed,
                      Constants.IDs.kRearLeftModuleOffset),
            Swerve_Module_Position.BACK_RIGHT,
                  new SwerveModule(
                    Swerve_Module_Position.BACK_RIGHT,
                      new CANSparkMax(IDs.kRearRightSteerMotor, CANSparkLowLevel.MotorType.kBrushless),
                      new CANSparkMax(IDs.kRearRightDriveMotor, CANSparkLowLevel.MotorType.kBrushless),
                      new CANcoder(IDs.kRearRightCanCoder),
                      Constants.DriveConstants.kBackRightTurningMotorReversed,
                      Constants.DriveConstants.kBackRightDriveMotorReversed,
                      Constants.IDs.kRearRightModuleOffset))); 


 private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

 //private Trajectory m_Trajectory;
 
 //private final SwerveDrivePoseEstimator m_odometry;
  //private double m_simYaw;
  //private DoublePublisher pitchPub, rollPub, yawPub, odometryXPub, odometryYPub, odometryYawPub;

  private boolean useHeadingTarget = false;
  //private double m_desiredRobotHeading;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(
          Constants.TrapezoidConstants.kMaxRotationRadiansPerSecond,
          Constants.TrapezoidConstants.kMaxRotationRadiansPerSecondSquared);
    //private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private final ProfiledPIDController m_rotationController =
      new ProfiledPIDController(
          Constants.TrapezoidConstants.kP_Rotation,
          Constants.TrapezoidConstants.kI_Rotation,
          Constants.TrapezoidConstants.kD_Rotation,
          m_constraints);
  private double m_rotationOutput;

  ChassisSpeeds chassisSpeeds;

  private double m_maxVelocity = Constants.TrapezoidConstants.kMaxSpeedMetersPerSecond;
 
  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain() {
    new Thread(()->{
    try{
      Thread.sleep(1000);
      zeroHeading();
    } catch (Exception e){
    }
    }).start();
  }

    /*m_odometry = new SwerveDrivePoseEstimator(Constants.ModulePosition.kSwerveKinematics,
      getHeadingRotation2d(), getSwerveDriveModulePositionsArray(), getPoseMeters());
      if (TimedRobot.isReal()) resetModulesToAbsolute();
    
  }*/

  //added 2/19
  public void zeroHeading(){
    m_gyro.reset();
  }


  

    //m_gyro.zeroYaw();
   // m_gyro.getRotation2d();

   


  /*private void resetModulesToAbsolute() {
    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
     module.resetEncoders();
  }*/

  
  
    public void drive(double throttle, double strafe, double rotation, boolean isFieldRelative, boolean isOpenLoop) {
    throttle *= m_maxVelocity;
    strafe *= m_maxVelocity;
    rotation *= Constants.TrapezoidConstants.kMaxRotationRadiansPerSecond;

    if (useHeadingTarget) {
       rotation = m_setpoint.velocity;
      rotation = m_rotationOutput;
       SmartDashboard.putNumber("Rotation Target", Units.radiansToDegrees(m_setpoint.position));
       SmartDashboard.putNumber("Rotation Speed ", Units.radiansToDegrees(rotation));
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, getHeadingRotation2d());
    } else {
      chassisSpeeds =
          isFieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  throttle, strafe, rotation, getHeadingRotation2d())
              : new ChassisSpeeds(throttle, strafe, rotation);
    }

   // Rotation2d getHeadingRot = getHeadingRotation2d(); // added 2/20 LC

   Map<Swerve_Module_Position, SwerveModuleState> moduleStates =
       ModuleMap.of(Constants.ModulePosition.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));

     SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), m_maxVelocity);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
     module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);      
  }
  
 
   /*public void setRobotHeading(double desiredAngleSetpoint) {
    m_desiredRobotHeading = desiredAngleSetpoint;
  }

  
/* 

  public void calculateRotationSpeed() {
    // m_goal = new TrapezoidProfile.State(Units.degreesToRadians(m_desiredRobotHeading), 0);
    // var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    // m_setpoint = profile.calculate(0.02);

    m_rotationOutput =
      m_rotationController.calculate(getHeadingRotation2d().getRadians(), m_desiredRobotHeading);
  }
   */
  
   public void enableHeadingTarget(boolean enable) {
    useHeadingTarget = enable;
  }

 
  /*public void resetState() {
    m_setpoint =
        new TrapezoidProfile.State();
            Units.degreesToRadians(getHeadingDegrees()); Units.degreesToRadians(0);
  }
*/

 
  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocity);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  } 


   public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }



  public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
    var states = Constants.ModulePosition.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    setSwerveModuleStates(states, false);
  }



   /*public void setOdometry(Pose2d pose) {
    m_odometry.resetPosition(getHeadingRotation2d(), getSwerveDriveModulePositionsArray(), pose);
  }*/

  public void stopModules(){
    for (SwerveModule module : m_swerveModules.values()){
      module.stop();
    }

  }
  public double getPitchDegrees() {
    return m_gyro.getPitch();
  }

  public double getRollDegrees() {
    return m_gyro.getRoll();
  }

  public double getHeadingDegrees() {
    return -Math.IEEEremainder((m_gyro.getAngle()), 360);
  }
 
 public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }
 
  public boolean getModuleInitStatus() {
    for (Swerve_Module_Position i : m_swerveModules.keySet()) {
      if (!m_swerveModules.get(i).getInitSuccess()) {
        return false;
      }
    }
    return true;
  }

 /* public Pose2d getPoseMeters() {
    return m_odometry.getEstimatedPosition();
  }*/

  public SwerveModule getSwerveModule(Swerve_Module_Position modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public Map<Swerve_Module_Position, SwerveModuleState> getModuleStates() {
    Map<Swerve_Module_Position, SwerveModuleState> map = new HashMap<>();
    for (Swerve_Module_Position i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getState());
    return map;
  }

  public Map<Swerve_Module_Position, SwerveModulePosition> getModulePositions() {
    Map<Swerve_Module_Position, SwerveModulePosition> map = new HashMap<>();
    for (Swerve_Module_Position i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getPosition());
    return map;
  }

  public SwerveModulePosition[] getSwerveDriveModulePositionsArray() {
    return ModuleMap.orderedValues(getModulePositions(), new SwerveModulePosition[0]);
  }

  public Map<Swerve_Module_Position, Pose2d> getModulePoses() {
    Map<Swerve_Module_Position, Pose2d> map = new HashMap<>();
    for (Swerve_Module_Position i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getModulePose());
    return map;
  }


  public void setNeutralMode(IdleMode mode) {
    for (SwerveModule module : m_swerveModules.values()) {
      module.setDriveNeutralMode(mode);
      module.setTurnNeutralMode(mode);
    }
  }

  /*public void setMaxVelocity(double mps) {
    m_maxVelocity = mps;
  }

  public void setCurrentTrajectory(Trajectory trajectory) {
    m_Trajectory = trajectory;
  }

  public Trajectory getCurrentTrajectory() {
    return m_Trajectory;
  }

  public SwerveDrivePoseEstimator getOdometry() {
  return m_odometry;
  }
*/
  public void resetGyro() {
    m_gyro.zeroYaw();
    m_gyro.reset();
  }

  /*public void updateOdometry() {
    m_odometry.update(getHeadingRotation2d(), getSwerveDriveModulePositionsArray());

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
      Transform2d moduleTransform =
          new Transform2d(
              Constants.ModulePosition.kModuleTranslations.get(module.getModulePosition()),
              module.getHeadingRotation2d());
      module.setModulePose(getPoseMeters().transformBy(moduleTransform));

    }
  }*/

  private void updateSmartDashboard(){
    SmartDashboard.putNumber("Pitch Pub", getPitchDegrees());
    SmartDashboard.putNumber("Roll Pub", getRollDegrees());
    SmartDashboard.putNumber("Yaw Pub", getHeadingDegrees());
    SmartDashboard.putNumber("Robot Heading", getHeadingDegrees()); //fix
  }

  public void disabledPeriodic() {

  }
  
  @Override
  public void periodic() {
   
    //updateOdometry();
    updateSmartDashboard();
  }


  

}




