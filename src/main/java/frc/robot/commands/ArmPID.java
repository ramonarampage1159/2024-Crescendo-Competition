// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
//import frc.robot.Constants;
import frc.robot.RobotContainer;
 
public class ArmPID extends Command {
  public ArmPID() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
    //addRequirements(RobotContainer.m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operatorController.getRawButtonPressed(Constants.OperatorController.JoystickButtons.m_yButton)) {
      double ArmDownPValue = Constants.ArmDownPIDCoeffcients.m_ArmDownP;
      double ArmDownIValue = Constants.ArmDownPIDCoeffcients.m_ArmDownI;
      double ArmDownDValue = Constants.ArmDownPIDCoeffcients.m_ArmDownD;
      RobotContainer.m_arm.setPIDValues(ArmDownPValue, ArmDownIValue, ArmDownDValue, 
      Constants.ArmDownPIDCoeffcients.m_ArmDownMinOutput, Constants.ArmDownPIDCoeffcients.m_ArmDownMaxOutput);
      double ArmDownRotations = Constants.ArmDownPIDCoeffcients.m_ArmDownRotations;
      RobotContainer.m_arm.setArmReference(ArmDownRotations, CANSparkBase.ControlType.kPosition);
    }

    if(RobotContainer.operatorController.getRawButtonPressed(Constants.OperatorController.JoystickButtons.m_aButton)){
      double ArmUpPValue = Constants.ArmUpPIDCoefficients.m_ArmUpP;
      double ArmUpIValue = Constants.ArmUpPIDCoefficients.m_ArmUpI;
      double ArmUpDValue = Constants.ArmUpPIDCoefficients.m_ArmUpD;
      RobotContainer.m_arm.setPIDValues(ArmUpPValue, ArmUpIValue, ArmUpDValue, 
      Constants.ArmUpPIDCoefficients.m_ArmUpMinOutput, Constants.ArmUpPIDCoefficients.m_ArmUpMaxOutput);
      double ArmUpRotations = Constants.ArmUpPIDCoefficients.m_ArmUpRotations;
      RobotContainer.m_arm.setArmReference(ArmUpRotations, CANSparkBase.ControlType.kPosition);
    }
   
    if(RobotContainer.operatorController.getRawButtonPressed(Constants.OperatorController.JoystickButtons.m_bButton)){
      double ArmAmpPValue = Constants.ArmPIDAmpCoefficients.m_AmpP;
      double ArmAmpIValue = Constants.ArmPIDAmpCoefficients.m_AmpI;
      double ArmAmpDValue = Constants.ArmPIDAmpCoefficients.m_AmpD;
      RobotContainer.m_arm.setPIDValues(ArmAmpPValue, ArmAmpIValue, ArmAmpDValue, 
      Constants.ArmPIDAmpCoefficients.m_AmpMinOutput, Constants.ArmPIDAmpCoefficients.m_AmpMaxOutput);
      double ArmAmpRotations = Constants.ArmPIDAmpCoefficients.m_AmpRotations;
      RobotContainer.m_arm.setArmReference(ArmAmpRotations, CANSparkBase.ControlType.kPosition);
    }

    if(RobotContainer.operatorController.getRawButtonPressed(Constants.OperatorController.JoystickButtons.m_xButton)){
      double ArmPodiumPValue = Constants.ArmPodiumPIDCoeffiecients.m_PodiumP;
      double ArmPodiumIValue = Constants.ArmPodiumPIDCoeffiecients.m_PodiumI;
      double ArmPodiumDValue = Constants.ArmPodiumPIDCoeffiecients.m_PodiumD;
      RobotContainer.m_arm.setPIDValues(ArmPodiumPValue, ArmPodiumIValue, ArmPodiumDValue, 
      Constants.ArmPodiumPIDCoeffiecients.m_PodiumMinOutput, Constants.ArmPodiumPIDCoeffiecients.m_PodiumMaxOutput);
      double ArmPodiumRotations = Constants.ArmPodiumPIDCoeffiecients.m_PodiumRotations;
      RobotContainer.m_arm.setArmReference(ArmPodiumRotations, CANSparkBase.ControlType.kPosition);
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
