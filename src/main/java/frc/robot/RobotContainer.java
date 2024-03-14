// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.ArmPID;
import frc.robot.commands.Climber;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.swerve.SwerveDriver;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.commands.auto.AutoCmds.AutonArmAmp;
import frc.robot.commands.auto.AutoCmds.AutonArmDown;
import frc.robot.commands.auto.AutoCmds.AutonArmPodium;
import frc.robot.commands.auto.AutoCmds.AutonArmUp;
import frc.robot.commands.auto.AutoCmds.AutonIntake;
import frc.robot.commands.auto.AutoCmds.AutonShoot;
import frc.robot.commands.auto.AutoCmds.AutonStopIntake;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static Joystick driverController = new Joystick(Constants.DriverController.m_driverController);
  public static Joystick operatorController = new Joystick(Constants.OperatorController.m_operatorController);

  private final SendableChooser<Command> autoChooser;

  HashMap<String, Command> m_eventMap = new HashMap<>();
  
  public static SwerveDrivetrain m_swerveDrive = new SwerveDrivetrain();

  public static IntakeSubsystem m_intake = new IntakeSubsystem();

  public static ClimbSubsystem m_climb = new ClimbSubsystem();

  public static ArmSubsystem m_arm = new ArmSubsystem();



  public static AutonShoot m_autoshoot = new AutonShoot();

  public static AutonIntake m_autoIntake = new AutonIntake();

  public static AutonStopIntake m_autoStopInt = new AutonStopIntake();

  public static AutonArmDown m_autoArmDown = new AutonArmDown();

  public static AutonArmUp m_autoArmUp = new AutonArmUp();

  public static AutonArmPodium m_autoArmPodium = new AutonArmPodium();

  public static AutonArmAmp m_autoArmAmp = new AutonArmAmp();

  
  public RobotContainer() {

    // setting named cmds
    NamedCommands.registerCommand("shoot note", m_autoshoot);
    NamedCommands.registerCommand("intake note", m_autoIntake);
    NamedCommands.registerCommand("motors stop", m_autoStopInt);
    NamedCommands.registerCommand("arm down", m_autoArmDown);
    NamedCommands.registerCommand("arm up", m_autoArmUp);
    NamedCommands.registerCommand("arm amp", m_autoArmAmp);
    NamedCommands.registerCommand("arm podium", m_autoArmPodium);


    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

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

   m_arm.setDefaultCommand(new ArmPID());

   m_climb.setDefaultCommand(new Climber());

   m_intake.setDefaultCommand(new IntakeCommand());

   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
