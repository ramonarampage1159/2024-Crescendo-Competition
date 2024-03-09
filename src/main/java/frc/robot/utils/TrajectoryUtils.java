// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtils {

  public static FollowPathHolonomic generatePPHolonomicCommand(
          SwerveDrivetrain swerveDrive, String pathName, double maxSpeed) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return generatePPHolonomicCommand(swerveDrive, path, maxSpeed);
  }

  public static FollowPathHolonomic generatePPHolonomicCommand(
          SwerveDrivetrain swerveDrive,
          PathPlannerPath path,
          double maxSpeed) {
    return new FollowPathHolonomic(
            path,
            swerveDrive::getPoseMeters,
            swerveDrive::getChassisSpeeds,
            swerveDrive::setChassisSpeed,
            new HolonomicPathFollowerConfig(
                    new PIDConstants(Constants.TrapezoidConstants.kP_Translation, Constants.TrapezoidConstants.kI_Translation, Constants.TrapezoidConstants.kD_Translation),
                    new PIDConstants(Constants.TrapezoidConstants.kP_Rotation, Constants.TrapezoidConstants.kI_Rotation, Constants.TrapezoidConstants.kD_Rotation),
                    maxSpeed,
                    0.86210458762,
                    new ReplanningConfig(false, false, 1.0, 0.25)),
            () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
            swerveDrive);
  }
}