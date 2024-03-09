// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.ArmPID;
import frc.robot.commands.Climber;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.autos.DriveForward;
import frc.robot.commands.swerve.SwerveDriver;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static Joystick driverController = new Joystick(Constants.DriverController.m_driverController);
  public static Joystick operatorController = new Joystick(Constants.OperatorController.m_operatorController);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  HashMap<String, Command> m_eventMap = new HashMap<>();
  
  public static SwerveDrivetrain m_swerveDrive = new SwerveDrivetrain();

  public static IntakeSubsystem m_intake = new IntakeSubsystem();

  public static ClimbSubsystem m_climb = new ClimbSubsystem();

  public static ArmSubsystem m_arm = new ArmSubsystem();

  //public static WristSubsystem m_wrist = new WristSubsystem();


  
  
  public RobotContainer() {
        initAutoBuilder();
        initializeAutoChooser();

    configureBindings();
  }

  public void initializeAutoChooser(){

    m_chooser.setDefaultOption("no auto", null);
        //m_chooser.addOption();
        //m_chooser.addOption("2 Note Auto", new frc.robot.commands.auto.AutonSequences.TwoNoteAuto());

        m_chooser.addOption(
       "DriveForward", new DriveForward("DriveForward", m_swerveDrive));

        SmartDashboard.putData("Auto Choices:", m_chooser);

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

  private void initAutoBuilder() {
    m_eventMap.put("wait", new WaitCommand(2));

    // See https://pathplanner.dev/pplib-build-an-auto.html#configure-autobuilder for an explanation
    AutoBuilder.configureHolonomic(
            m_swerveDrive::getPoseMeters,
            m_swerveDrive::setOdometry,
            m_swerveDrive::getChassisSpeeds,
            m_swerveDrive::setChassisSpeed,
            new HolonomicPathFollowerConfig(
                    new PIDConstants(Constants.TrapezoidConstants.kP_Translation, Constants.TrapezoidConstants.kI_Translation, Constants.TrapezoidConstants.kD_Translation),
                    new PIDConstants(Constants.TrapezoidConstants.kP_Rotation, Constants.TrapezoidConstants.kI_Rotation, Constants.TrapezoidConstants.kD_Rotation),
                    Constants.TrapezoidConstants.kMaxSpeedMetersPerSecond,
                    0.86210458762,
                    new ReplanningConfig()
            ),
            ()-> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
            m_swerveDrive
    );
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
    return m_chooser.getSelected();
  }
}
