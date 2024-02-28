package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class PidSlot {

  public static void setTurnMotorConfig(CANSparkMax motorController) {
    motorController.getPIDController().setFF(0.0, PID_SLOT.POS_SLOT.ordinal());
    motorController.getPIDController().setP(0.2, PID_SLOT.POS_SLOT.ordinal());
    motorController.getPIDController().setI(0.0, PID_SLOT.POS_SLOT.ordinal());
    motorController.getPIDController().setD(0.0, PID_SLOT.POS_SLOT.ordinal());

    motorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
    motorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
    motorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 50);

    motorController.setSmartCurrentLimit(30, 15);
    //motorController.setSmartCurrentLimit(40);

    motorController.enableSoftLimit(SoftLimitDirection.kForward, false);
    motorController.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public static void setDriveMotorConfig(CANSparkMax motorController) {
    motorController.getPIDController().setFF(0.21, PID_SLOT.VEL_SLOT.ordinal());
    motorController.getPIDController().setP(0.235, PID_SLOT.VEL_SLOT.ordinal());
    motorController.getPIDController().setI(0.0, PID_SLOT.VEL_SLOT.ordinal());
    motorController.getPIDController().setD(0.0, PID_SLOT.VEL_SLOT.ordinal());

    motorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);//10
    motorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);//20
    motorController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 50);//50

    motorController.setSmartCurrentLimit(25, 10);
    //motorController.setSmartCurrentLimit(60);

    //motorController.setOpenLoopRampRate(0.2);
    
    motorController.getPIDController().setOutputRange(-1, 1);
    motorController.enableSoftLimit(SoftLimitDirection.kForward, false);
    motorController.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }


  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  public enum PID_SLOT {
    POS_SLOT,
    VEL_SLOT,
    SIM_SLOT
  }
}
