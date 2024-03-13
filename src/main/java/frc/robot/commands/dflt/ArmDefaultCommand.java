// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.dflt;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDefaultCommand extends CommandBase {

  ArmSubsystem s_robotArm;

  /** Creates a new Arm Default Command. 
   * This command holds the arm at the desired location set in by the ArmSetPosCommand.
   * 
   * @param s_robotArm Robot arm subsystem
   */
  public ArmDefaultCommand(ArmSubsystem s_robotArm) {
    addRequirements(s_robotArm);
    this.s_robotArm = s_robotArm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((s_robotArm.getAngle() >= ArmPivotConstants.kExtendLoc && RobotContainer.l_armPos.getTargetEnum() == ArmSubsystem.ArmPos.kExtend) ||
       (s_robotArm.getAngle() <= ArmPivotConstants.kHomeLoc && RobotContainer.l_armPos.getTargetEnum() == ArmSubsystem.ArmPos.kHome)) {
      
      s_robotArm.stopMotors();
    }
    else {
      s_robotArm.setAngle(RobotContainer.l_armPos.getTargetPosition());
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
