// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.AutoCmds;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutonArmUp extends Command {
  /** Creates a new AutonArm. */
  public AutonArmUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      double AutoArmUpPValue = Constants.ArmUpPIDCoefficients.m_ArmUpP;
      double AutoArmUpIValue = Constants.ArmUpPIDCoefficients.m_ArmUpI;
      double AutoArmUpDValue = Constants.ArmUpPIDCoefficients.m_ArmUpD;
      RobotContainer.m_arm.setPIDValues(AutoArmUpPValue, AutoArmUpIValue, AutoArmUpDValue, 
      Constants.ArmUpPIDCoefficients.m_ArmUpMinOutput, Constants.ArmUpPIDCoefficients.m_ArmUpMaxOutput);
      double AutoArmUpRotations = Constants.ArmUpPIDCoefficients.m_ArmUpRotations;
      RobotContainer.m_arm.setArmReference(AutoArmUpRotations, CANSparkBase.ControlType.kPosition);
  
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
