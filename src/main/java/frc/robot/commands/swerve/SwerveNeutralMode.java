package frc.robot.commands.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

/** Sets the drivetrain to neutral (coast/brake) */
public class SwerveNeutralMode extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrivetrain m_swerveDrive;

  private final IdleMode m_mode;

  /**
   * Sets the drivetrain neutral mode (coast/brake).
   *
   * @param driveTrain The driveTrain used by this command.
   * @param mode {@link DriveTrainNeutralMode}: COAST, BRAKE, or HALF_BRAKE.
   */
  public SwerveNeutralMode(SwerveDrivetrain swerveDrive, IdleMode mode) {
    m_swerveDrive = swerveDrive;
    m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.setNeutralMode(m_mode);
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
    return true;
  }
}