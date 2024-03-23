// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPos;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousPickup extends ParallelCommandGroup {
  /** Creates a new AutonomousPickup. */
  public AutonomousPickup(ArmSubsystem s_robotArm, IntakeSubsystem s_intake, DrivetrainSubsystem s_robotDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new AutonomousArmSetPos(s_robotArm, ArmPos.kFloor),
    new AutonomousDriveDistanceDelayed(s_robotDrive, 2, 0.8),
    new AutonomousIntakeSetSpdDelayless(s_intake, 1, 3)
    );
  }
}
