// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.ModuleMap;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class IDs{
    public static final int kFrontLeftSteerMotor = 4;
    public static final int kFrontLeftDriveMotor = 5;
    public static final int kFrontLeftCanCoder = 11;
    public static final double kFrontLeftModuleOffset = -0.052978515625 * 360;

    public static final int kFrontRightSteerMotor = 6;
    public static final int kFrontRightDriveMotor = 7;
    public static final int kFrontRightCanCoder = 10;
    public static final double kFrontRightModuleOffset = -0.589111328125 * 360;

    public static final int kRearLeftSteerMotor = 2;
    public static final int kRearLeftDriveMotor = 3;
    public static final int kRearLeftCanCoder = 13;
    public static final double kRearLeftModuleOffset = -0.606689453125 * 360;

    public static final int kRearRightSteerMotor = 8;
    public static final int kRearRightDriveMotor = 9; 
    public static final int kRearRightCanCoder = 12;
    public static final double kRearRightModuleOffset = -0.58984375 * 360;
  } 
 

  public final class ZeroPIDCoefficients{
    public static final double m_ZerokP = 0; 
    public static final double m_ZerokI = 0.000;
    public static final double m_ZerokD = 0.000; 
    public static final double m_ZerokIz = 0; 
    public static final double m_ZerokFF = 0; 
    public static final double m_ZerokMaxOutput = 0; 
    public static final double m_ZerokMinOutput = -0;
    


  }

  public final class DriverController{

    public static final int m_driverController = 0;
       
      public final class Joystick{

       public static final int m_leftStickX = 0;
       public static final int m_leftStickY = 1;
       public static final int m_rightStickX = 2;
    }

    public final class DriverButtons {
      public static final int m_bDriveButton = 3;
      public static final int m_leftDriveBumper = 5; 
      public static final int m_leftDriveTrigger = 7;
      public static final int m_rightDriveTrigger = 8; 

    }
    
   }
  
   public final class OperatorController{

    public static final int m_operatorController = 1; 

      public final class JoystickButtons{

        public static final int m_xButton = 1;
        public static final int m_aButton = 2;
        public static final int m_bButton = 3;
        public static final int m_yButton = 4; 
        public static final int m_leftBumper = 5; 
        public static final int m_rightBumper = 6;
        public static final int m_leftTrigger = 7;
        public static final int m_rightTrigger = 8; 
        public static final int m_backButton = 9;
        public static final int m_startButton = 10;
        public static final int m_leftStickButton = 11;
        public static final int m_rightStickButton = 12;

      }

      public final class Joystick{

       public static final int m_leftStickX = 0;
       public static final int m_leftStickY = 1;
       public static final int m_rightStickX = 2;
    }
    
   }

   public static final class ClimbMotor{
    public static final int m_climbMotor1 = 24;

   }

   public static final class WristMotors{
    public static final int m_wristMotorLeft = 17;
    public static final int m_wristMotorRight = 18;
   }

   public static final class ArmMotors{
    public static final int m_armMotorRight = 19;
    public static final int m_armMotorLeft = 20;
   }

   public static final class IntakeMotors{
    public static final int m_topMotor = 21;
    public static final int m_bottomMotor = 22;
    public static final int m_transferMotor = 23; 
   }



   public static final class DriveConstants{
    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kFrontLeftDriveMotorReversed = true; 

    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = true;
    
    
    public static final boolean kBackLeftTurningMotorReversed = true;
    public static final boolean kBackLeftDriveMotorReversed = true; 

    public static final boolean kBackRightTurningMotorReversed = true;
    public static final boolean kBackRightDriveMotorReversed = true;
    

    public enum SwerveDriveModulePosition {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }
  }


  public static final class ModulePosition{

    public static final double kTrackWidth = Units.inchesToMeters(22.25);
    public static final double kWheelBase = Units.inchesToMeters(25.25);
    
    public static final Map<Swerve_Module_Position, Translation2d> kModuleTranslations = Map.of(
        Swerve_Module_Position.FRONT_LEFT, new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Swerve_Module_Position.FRONT_RIGHT, new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Swerve_Module_Position.BACK_LEFT, new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Swerve_Module_Position.BACK_RIGHT, new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));

    
      public static final boolean kGyroReversed = true;

    public enum Swerve_Module_Position {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }

   }

   public static final class ModuleConstants {

    // ModuleConfiguration MK4I_L2

    //150/7:1

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static double mk4iL2DriveGearRatio = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)); //6.75 : 1

    public static double mk4iL2TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0)); //21.43

    public static final double kDriveMetersPerEncRev =

        (kWheelDiameterMeters * Math.PI) / mk4iL2DriveGearRatio;

    public static final double kDriveEncRPMperMPS = kDriveMetersPerEncRev / 60;

    public static double kEncoderRevsPerMeter = 1 / kDriveMetersPerEncRev;

    public static double kFreeMetersPerSecond = 5600 * kDriveEncRPMperMPS;
  
    public static final double kTurningDegreesPerEncRev =

        360 / mk4iL2TurnGearRatio;

    public static final double kTurningEncRPMtoDPS = kTurningDegreesPerEncRev / 60;

    
    public static final double kPModuleTurningController = .025;

    public static final double kPModuleDriveController = .2;

    
    // use sysid on robot??
    public static double ksVolts = .055;
    public static double kvVoltSecondsPerMeter = .2;
    public static double kaVoltSecondsSquaredPerMeter = .02;

    public static double kPModuleTurnController;

    public static double kSMmaxAccel = 90;//deg per sec per sec

    public static double maxVel= 90; // deg per sec

    public static double allowedErr = .05;//deg

    // sysid on module?
    public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;
    // sysid on module?
    public static final double kvTurnVoltSecondsPerRadian = 1.47; 
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; 

    
    public static double kMaxModuleAngularSpeedDegPerSec = 90;

    public static final double kMaxModuleAngularAccelerationDegreesPerSecondSquared = 90;

    public static final double kDriveRevToMeters =
      ((kWheelDiameterMeters * Math.PI) / mk4iL2DriveGearRatio);
    public static final double kDriveRpmToMetersPerSecond =
      kDriveRevToMeters / 60.0;
    public static final double kTurnRotationsToDegrees =
      360.0 / mk4iL2TurnGearRatio;
    
   }
  
    public static final class TrapezoidConstants {
    
      public static final double kMaxSpeedMetersPerSecond = 3;
  
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;

      private static final double DiagonalRadius = Math.sqrt(Math.pow(22.25*.0254 / 2,2) + Math.pow(25.25*.0254/2, 2)); 
  
      public static final double kMaxRotationRadiansPerSecond = kMaxSpeedMetersPerSecond / DiagonalRadius;
      public static final double kMaxRotationRadiansPerSecondSquared = kMaxRotationRadiansPerSecond * 2; 
  
      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;
  
      public static final double kP_Translation = 0.01;
      public static final double kI_Translation = 0;
      public static final double kD_Translation = 0;
      public static final double kP_Rotation = 0.01;
      public static final double kI_Rotation = 0;
      public static final double kD_Rotation = 0.01;
      
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
          kMaxRotationRadiansPerSecond, kMaxRotationRadiansPerSecondSquared);

  }
   


}
