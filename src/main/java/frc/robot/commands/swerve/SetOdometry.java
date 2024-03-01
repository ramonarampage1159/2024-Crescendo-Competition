// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class SetOdometry extends Command {


  private Pose2d  m_pose2d;
  
  public SetOdometry(SwerveDrivetrain swerveDrive, Pose2d pose2d) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveDrive);
    
    m_pose2d = pose2d;
  }

  
  @Override
  public void initialize() {
    SmartDashboard.putNumber("SwerveInitialPositionX", m_pose2d.getX());
    SmartDashboard.putNumber("SwerveInitialPositionY", m_pose2d.getY());
    SmartDashboard.putNumber("SwerveInitialPositionRotation", m_pose2d.getRotation().getDegrees());
  }
  
  
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
