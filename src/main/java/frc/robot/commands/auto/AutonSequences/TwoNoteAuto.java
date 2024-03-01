// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.AutonSequences;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteAuto extends SequentialCommandGroup {
  /** Creates a new TwoNoteAuto. */
  public TwoNoteAuto() {

    final SwerveDrivetrain swerveSubsystem = new SwerveDrivetrain();

    addRequirements(RobotContainer.m_swerveDrive);

     TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                Constants.TrapezoidConstants.kMaxSpeedMetersPerSecond,
                Constants.TrapezoidConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.ModulePosition.kSwerveKinematics);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        PIDController xController = new PIDController(Constants.TrapezoidConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.TrapezoidConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.TrapezoidConstants.kPThetaController, 0, 0, Constants.TrapezoidConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
              
              
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
   
  }
}
