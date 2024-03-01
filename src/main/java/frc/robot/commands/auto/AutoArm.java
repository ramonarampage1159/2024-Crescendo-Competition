// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoArm extends Command {
  /** Creates a new AutoArm. */

  double m_pValue;
  double m_rotations;

  public AutoArm(double pValue, double rotations) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);

    m_pValue = pValue;
    m_rotations = rotations;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_arm.setAutoPIDValues(m_pValue);
    RobotContainer.m_arm.setArmReference(m_rotations, CANSparkMax.ControlType.kPosition);

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
