// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.AutoCmds;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutonArmPodium extends Command {
  /** Creates a new AutonArmPodium. */
  public AutonArmPodium() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      double AutoArmPodiumPValue = Constants.ArmPodiumPIDCoeffiecients.m_PodiumP;
      double AutoArmPodiumIValue = Constants.ArmPodiumPIDCoeffiecients.m_PodiumI;
      double AutoArmPodiumDValue = Constants.ArmPodiumPIDCoeffiecients.m_PodiumD;
      RobotContainer.m_arm.setPIDValues(AutoArmPodiumPValue, AutoArmPodiumIValue, AutoArmPodiumDValue, 
      Constants.ArmPodiumPIDCoeffiecients.m_PodiumMinOutput, Constants.ArmPodiumPIDCoeffiecients.m_PodiumMaxOutput);
      double AutoArmPodiumRotations = Constants.ArmPodiumPIDCoeffiecients.m_PodiumRotations;
      RobotContainer.m_arm.setArmReference(AutoArmPodiumRotations, CANSparkBase.ControlType.kPosition);
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
