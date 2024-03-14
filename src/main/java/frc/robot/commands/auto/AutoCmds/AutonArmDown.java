// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.AutoCmds;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutonArmDown extends Command {
  /** Creates a new AutonArmDown. */

  public AutonArmDown() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      double AutoArmDownPValue = Constants.ArmDownPIDCoeffcients.m_ArmDownP;
      double AutoArmDownIValue = Constants.ArmDownPIDCoeffcients.m_ArmDownI;
      double AutoArmDownDValue = Constants.ArmDownPIDCoeffcients.m_ArmDownD;
      RobotContainer.m_arm.setPIDValues(AutoArmDownPValue, AutoArmDownIValue, AutoArmDownDValue, 
      Constants.ArmDownPIDCoeffcients.m_ArmDownMinOutput, Constants.ArmDownPIDCoeffcients.m_ArmDownMaxOutput);
      double AutoArmDownRotations = Constants.ArmDownPIDCoeffcients.m_ArmDownRotations;
      RobotContainer.m_arm.setArmReference(AutoArmDownRotations, CANSparkBase.ControlType.kPosition);

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
