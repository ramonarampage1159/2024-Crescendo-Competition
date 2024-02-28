// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
//import frc.robot.Robot;
//import frc.robot.RobotContainer;

public class SwerveDriver extends Command {
  /** Creates a new SwerveDriver. */

  private final SwerveDrivetrain m_swerveDrive;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

  public SwerveDriver(
    SwerveDrivetrain swerveDriveSubsystem,
    DoubleSupplier throttleInput,
    DoubleSupplier strafeInput,
    DoubleSupplier rotationInput) {

      m_swerveDrive = swerveDriveSubsystem;
      m_throttleInput = throttleInput;
      m_strafeInput = strafeInput;
      m_rotationInput = rotationInput;

    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

double throttle =
        MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()), 0.06)
            * Math.signum(m_throttleInput.getAsDouble());
    double strafe =
        MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()), 0.06)
            * Math.signum(m_strafeInput.getAsDouble());
    double rotation =
        MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()), 0.06)
            * Math.signum(m_rotationInput.getAsDouble());
            
      System.out.println(throttle);
      System.out.println(strafe);
      System.out.print(rotation);
  
    m_swerveDrive.drive(throttle, strafe, rotation, true, false);
  
  
  
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
