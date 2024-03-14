// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.AutoCmds;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutonArmAmp extends Command {
  /** Creates a new AutonArmAmp. */
  public AutonArmAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double AutoArmAmpPValue = Constants.ArmPIDAmpCoefficients.m_AmpP;
      double AutoArmAmpIValue = Constants.ArmPIDAmpCoefficients.m_AmpI;
      double AutoArmAmpDValue = Constants.ArmPIDAmpCoefficients.m_AmpD;
      RobotContainer.m_arm.setPIDValues(AutoArmAmpPValue, AutoArmAmpIValue, AutoArmAmpDValue, 
      Constants.ArmPIDAmpCoefficients.m_AmpMinOutput, Constants.ArmPIDAmpCoefficients.m_AmpMaxOutput);
      double AutoArmAmpRotations = Constants.ArmPIDAmpCoefficients.m_AmpRotations;
      RobotContainer.m_arm.setArmReference(AutoArmAmpRotations, CANSparkBase.ControlType.kPosition);
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
