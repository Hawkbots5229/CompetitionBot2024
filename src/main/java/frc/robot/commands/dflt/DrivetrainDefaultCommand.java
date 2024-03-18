// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.dflt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;;

public class DrivetrainDefaultCommand extends Command {
    // Slew rate limiters to make joystick inputs more gentle; 1/2 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(10);

    PIDController m_rotationPIDController = new PIDController(DriveConstants.rotKp, DriveConstants.rotKi, DriveConstants.rotKd);
    DrivetrainSubsystem drive;

  /** Creates a new Drivetrain Default Command. 
   * This command contorls the motion of the robot via the joysticks.
   * 
   * @param s_robotDrive Robot drivetrain subsystem
   */
  public DrivetrainDefaultCommand(DrivetrainSubsystem s_robotDrive) {
    addRequirements(s_robotDrive);
    drive = s_robotDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotationPIDController.setTolerance(DriveConstants.rotToleranceDeg, DriveConstants.rotToleranceVel);
    m_rotationPIDController.setIntegratorRange(0, .5);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
     var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), DriveConstants.stickDeadband))
            * DriveConstants.maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
     var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), DriveConstants.stickDeadband))
            * DriveConstants.maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
     var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_driverController.getRightX(), DriveConstants.stickDeadband))
            * DriveConstants.maxAngularSpeed;
            
    RobotContainer.m_robotDrive.drive(xSpeed * DriveConstants.speedScale, ySpeed * DriveConstants.speedScale, rot*DriveConstants.rotationScale,RobotContainer.m_robotDrive.isFieldRelative,true);
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