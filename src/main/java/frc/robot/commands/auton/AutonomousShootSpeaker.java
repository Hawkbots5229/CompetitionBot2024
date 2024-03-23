// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem.ArmPos;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousShootSpeaker extends ParallelCommandGroup {
  /** Creates a new AutonomousShootSpeaker. */
  public AutonomousShootSpeaker(IntakeSubsystem s_intake, ShooterSubsystem s_robotShooter, ArmSubsystem s_robotArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousArmSetPos(s_robotArm, ArmPos.kFloor),
      new AutonomousShootSetSpd(s_robotShooter, ShooterSubsystem.shootDir.kOut, 5),
      new AutonomousIntakeSetSpd(s_intake, -0.8, 5)
    );
  }
}
