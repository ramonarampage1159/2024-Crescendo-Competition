// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climber extends Command {
  /** Creates a new Climber. */
  public Climber() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.driverController.getRawButtonPressed(Constants.DriverController.DriverButtons.m_leftDriveBumper)){
      RobotContainer.m_climb.solenoidOpen();

    }else if(RobotContainer.driverController.getRawButtonPressed(Constants.DriverController.DriverButtons.m_leftDriveTrigger)){
      RobotContainer.m_climb.solenoidClose();

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
