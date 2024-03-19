// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbSetSpdCommand extends Command {

  private final ClimbSubsystem s_robotClimb;
  private final double speed;
  

  /** Creates a new ClimbSetSpdCommand. */
  public ClimbSetSpdCommand(ClimbSubsystem s_robotClimb, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotClimb);
    this.s_robotClimb = s_robotClimb;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_robotClimb.setTargetOutput(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_robotClimb.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
