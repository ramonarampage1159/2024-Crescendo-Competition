// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climber extends Command {
  public Climber() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.m_climb.ZeroClimber();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.driverController.getRawButton(Constants.DriverController.DriverButtons.m_rightDriveTrigger)){
      RobotContainer.m_climb.ClimbExtend();
      System.out.println("extend");

    }else if(RobotContainer.driverController.getRawButton(Constants.DriverController.DriverButtons.m_leftDriveTrigger)){
      RobotContainer.m_climb.ClimbRetract();
      System.out.println("retract");


    }else{
      RobotContainer.m_climb.ClimbStop();
      System.out.println("stop");

    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_climb.ClimbStop();
    System.out.println("cmd ended");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
