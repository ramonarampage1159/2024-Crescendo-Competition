package frc.robot.commands.autos;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.SwerveNeutralMode;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.utils.TrajectoryUtils;

public class DriveForward extends SequentialCommandGroup {
  public DriveForward(
      String autoName,
      SwerveDrivetrain swerveDrive) {

    var autoPath = AutoBuilder.buildAuto(autoName);

    addCommands(
        autoPath,
        new SwerveNeutralMode(swerveDrive, IdleMode.kBrake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}