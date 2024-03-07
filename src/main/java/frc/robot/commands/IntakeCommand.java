// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operatorController.getRawButtonPressed(Constants.OperatorController.JoystickButtons.m_leftTrigger)){
      RobotContainer.m_intake.IntakeNote();

    }else if(RobotContainer.operatorController.getRawButtonReleased(Constants.OperatorController.JoystickButtons.m_leftTrigger)){
      RobotContainer.m_intake.stopMotors();
      RobotContainer.m_intake.stopTransfer();


    }else if(RobotContainer.operatorController.getRawButtonPressed(Constants.OperatorController.JoystickButtons.m_rightTrigger)){
      RobotContainer.m_intake.ShootNote();

    }else if(RobotContainer.operatorController.getRawButtonReleased(Constants.OperatorController.JoystickButtons.m_rightTrigger)){
      RobotContainer.m_intake.stopMotors();
      RobotContainer.m_intake.stopTransfer();

      
    }if(RobotContainer.operatorController.getRawButtonPressed(Constants.OperatorController.JoystickButtons.m_xButton)){
      RobotContainer.m_intake.ShootNoteAmp();
    }else if (RobotContainer.operatorController.getRawButtonReleased(Constants.OperatorController.JoystickButtons.m_xButton)){
      RobotContainer.m_intake.stopMotors();
      RobotContainer.m_intake.stopTransfer();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
