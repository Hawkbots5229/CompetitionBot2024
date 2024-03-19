// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.dflt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootDefaultCommand extends Command {

  private final ShooterSubsystem s_robotShoot;

  /** Creates a new ShootDefaultCommand. */
  public ShootDefaultCommand(ShooterSubsystem s_robotShoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotShoot);
    this.s_robotShoot = s_robotShoot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_robotShoot.stopMotors();
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
