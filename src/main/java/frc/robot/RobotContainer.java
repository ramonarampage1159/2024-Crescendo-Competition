// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.swerve.SwerveDriver;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final Joystick driverController = new Joystick(Constants.DriverController.m_driverController);
  public final Joystick operatorController = new Joystick(Constants.OperatorController.m_operatorController);

  HashMap<String, Command> m_eventMap = new HashMap<>();
  
  public static SwerveDrivetrain m_swerveDrive = new SwerveDrivetrain();

  
  
  public RobotContainer() {
    // Configure the trigger bindin
    //initializeSubsystems();

    configureBindings();
  }

    public double GetDriverRawAxis(int axis){
    return driverController.getRawAxis(axis);
     
  }
  
  public boolean getRawButtonPressed(int button){
    return operatorController.getRawButton(button);
  }
    
  public double GetOperatorRawAxis(int axis){
    return operatorController.getRawAxis(axis);
     
  }
  

  
  private void configureBindings() {
   m_swerveDrive.setDefaultCommand( new SwerveDriver(m_swerveDrive,
      ()-> driverController.getRawAxis(Constants.DriverController.Joystick.m_leftStickY), 
      ()-> driverController.getRawAxis(Constants.DriverController.Joystick.m_leftStickX),
      ()-> driverController.getRawAxis(Constants.DriverController.Joystick.m_rightStickX)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
