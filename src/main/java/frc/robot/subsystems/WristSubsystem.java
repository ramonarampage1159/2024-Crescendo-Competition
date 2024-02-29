// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
   WPI_TalonSRX m_wristMotorLeft = new WPI_TalonSRX(Constants.WristMotors.m_wristMotorLeft);
   WPI_TalonSRX m_wristMotorRight = new WPI_TalonSRX(Constants.WristMotors.m_wristMotorRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
