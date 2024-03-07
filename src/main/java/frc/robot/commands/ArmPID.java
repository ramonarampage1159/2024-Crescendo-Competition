// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
//import frc.robot.Constants;
import frc.robot.RobotContainer;
 
public class ArmPID extends Command {
  public ArmPID() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
    //addRequirements(RobotContainer.m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.operatorController.getRawButtonPressed(Constants.OperatorController.JoystickButtons.m_aButton)) {
      System.out.println("PushButtonForward");
      RobotContainer.m_arm.pushButtonForward();
    }
    else if(RobotContainer.operatorController.getRawButtonReleased(Constants.OperatorController.JoystickButtons.m_aButton)){
      System.out.println("StopMotors Forward");
       RobotContainer.m_arm.stopMotors();
    }
    if(RobotContainer.operatorController.getRawButtonPressed(Constants.OperatorController.JoystickButtons.m_yButton)){
      System.out.println("PushButtonBack");
      RobotContainer.m_arm.pushButtonBackward();
    }
    else if (RobotContainer.operatorController.getRawButtonReleased(Constants.OperatorController.JoystickButtons.m_yButton)){
         System.out.println("StopMotors Back");
         RobotContainer.m_arm.stopMotors();
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
