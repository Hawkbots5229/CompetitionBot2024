// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSetSpdCommand extends Command {

  private final ShooterSubsystem s_robotShooter;
  private final double speed;

  /** Creates a new ShootSetSpdCommand. */
  public ShootSetSpdCommand(ShooterSubsystem s_robotShooter, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotShooter);
    this.s_robotShooter = s_robotShooter;  
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_robotShooter.setTargetOutput(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_robotShooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
