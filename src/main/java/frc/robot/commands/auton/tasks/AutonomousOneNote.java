// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.tasks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.auton.AutonomousArmSetPos;
import frc.robot.commands.auton.AutonomousDriveDistance;
import frc.robot.commands.auton.AutonomousDriveStop;
//import frc.robot.commands.auton.AutonomousIntakeSetSpd;
import frc.robot.commands.auton.AutonomousResetEncoders;
import frc.robot.commands.auton.AutonomousShootSetSpd;
import frc.robot.commands.auton.AutonomousShootSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPos;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousOneNote extends SequentialCommandGroup {
  /** Creates a new AutonomousOneNote. */
  public AutonomousOneNote(ArmSubsystem s_robotArm, DrivetrainSubsystem s_robotDrive, IntakeSubsystem s_intake, ShooterSubsystem s_robotShooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ZeroHeading(s_robotDrive),
    new AutonomousResetEncoders(s_robotDrive),
    //new AutonomousArmSetPos(s_robotArm, ArmPos.kFloor),
    new AutonomousShootSpeaker(s_intake, s_robotShooter, s_robotArm),
    new AutonomousDriveDistance(s_robotDrive, 45, 0.8),
    new AutonomousDriveStop(s_robotDrive)

    );
  }
}
