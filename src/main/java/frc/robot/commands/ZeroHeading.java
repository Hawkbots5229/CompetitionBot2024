// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroHeading extends InstantCommand {

  private final DrivetrainSubsystem s_robotDrive;

  public ZeroHeading(DrivetrainSubsystem s_robotDrive) {

    this.s_robotDrive = s_robotDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_robotDrive.zeroHeading();
  }
}
