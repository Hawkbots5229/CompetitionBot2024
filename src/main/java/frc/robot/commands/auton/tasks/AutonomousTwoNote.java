// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.tasks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.AutonomousArmSetPos;
import frc.robot.commands.auton.AutonomousDriveDistance;
import frc.robot.commands.auton.AutonomousDriveStop;
import frc.robot.commands.auton.AutonomousIntakeSetSpd;
import frc.robot.commands.auton.AutonomousPickup;
import frc.robot.commands.auton.AutonomousResetEncoders;
import frc.robot.commands.auton.AutonomousShootSetSpd;
import frc.robot.commands.auton.AutonomousShootSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPos;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTwoNote extends SequentialCommandGroup {
  /** Creates a new AutonomousPickup. */
  public AutonomousTwoNote(DrivetrainSubsystem s_robotDrive, IntakeSubsystem s_intake, ArmSubsystem s_robotArm, ShooterSubsystem s_robotShooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new AutonomousResetEncoders(s_robotDrive),
    new AutonomousShootSpeaker(s_intake, s_robotShooter, s_robotArm),
    new AutonomousDriveDistance(s_robotDrive, 39, 0.8),
    new AutonomousPickup(s_robotArm, s_intake, s_robotDrive),
    new AutonomousDriveDistance(s_robotDrive, 41, -0.8),
    new AutonomousShootSpeaker(s_intake, s_robotShooter, s_robotArm)
    );
  }
}
