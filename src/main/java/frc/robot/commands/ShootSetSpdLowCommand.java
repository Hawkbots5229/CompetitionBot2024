// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSetSpdLowCommand extends Command {
  private final ShooterSubsystem s_robotShooter;
  private final ShooterSubsystem.shootDir direction;

  /** Creates a new ShootSetSpdLowCommand. */
  public ShootSetSpdLowCommand(ShooterSubsystem s_robotShooter, ShooterSubsystem.shootDir direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotShooter);
    this.s_robotShooter = s_robotShooter;
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(direction) {
      case kIn:
        s_robotShooter.wheelsInLow();
        break;
      case kOut:
        s_robotShooter.wheelsOutLow();
        break;
      case kOff:
        s_robotShooter.stopMotors();
        break;
      default:
        throw new AssertionError("Illegal value: " + direction);
    };
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
