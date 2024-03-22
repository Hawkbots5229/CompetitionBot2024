// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousShootSetSpd extends Command {

  private final Timer tmr = new Timer();
  private final ShooterSubsystem s_robotShoot;
  ShooterSubsystem.shootDir direction;
  private final double time;  

  /** Creates a new AutonomousShootSetSpd. */
  public AutonomousShootSetSpd(ShooterSubsystem s_robotShoot, ShooterSubsystem.shootDir direction, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotShoot);
    this.s_robotShoot = s_robotShoot;
    this.direction = direction;
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tmr.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(direction) {
      case kIn: 
        s_robotShoot.wheelsIn();
        break;
      case kOut: 
        s_robotShoot.wheelsOut();
        break;
      case kOff:
        s_robotShoot.stopMotors();
        break;
      default:
        throw new AssertionError("Illegal value: " + direction);   
    };
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_robotShoot.stopMotors();
    tmr.stop();
    tmr.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tmr.get() > time;
  }
}
