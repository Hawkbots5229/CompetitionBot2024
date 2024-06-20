// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.dflt.ArmDefaultCommand;
import frc.robot.commands.dflt.DrivetrainDefaultCommand;
import frc.robot.commands.dflt.IntakeDefaultCommand;
import frc.robot.commands.dflt.ShootDefaultCommand;
import frc.robot.library.ArmController;
import frc.robot.commands.ArmSetPosCommand;
import frc.robot.commands.ClimbSetSpdCommand;
import frc.robot.commands.IntakeSetSpdCommand;
import frc.robot.commands.ShootSetSpdCommand;
import frc.robot.commands.ShootSetSpdLowCommand;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.auton.tasks.AutonomousCrossLine;
import frc.robot.commands.auton.tasks.AutonomousDontMove;
import frc.robot.commands.auton.tasks.AutonomousOneNote;
import frc.robot.commands.auton.tasks.AutonomousTwoNote;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  // The robot's subsystems
  public static DrivetrainSubsystem m_robotDrive = new DrivetrainSubsystem();
  private DrivetrainDefaultCommand drivetrainDefaultCommand = new DrivetrainDefaultCommand(m_robotDrive);
  public static IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private IntakeDefaultCommand intakeDefaultCommand = new IntakeDefaultCommand(m_robotIntake);
  public static ArmSubsystem m_robotArm = new ArmSubsystem();
  private ArmDefaultCommand armDefaultCommand = new ArmDefaultCommand(m_robotArm);
  public static ArmController l_armPos = new ArmController(ArmSubsystem.ArmPos.kHome);
  public static ShooterSubsystem m_robotShoot = new ShooterSubsystem();
  private ShootDefaultCommand shootDefaultCommand = new ShootDefaultCommand(m_robotShoot);
  public static ClimbSubsystem m_robotClimb = new ClimbSubsystem();

  // The driver's controller
  public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static XboxController m_mechController = new XboxController(OIConstants.kMechControllerPort);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> sc_autonSelect = new SendableChooser<>();
  public RobotContainer() {

    // Create camera servers
    CameraServer.startAutomaticCapture("Claw Camera", 0);
    //CameraServer.startAutomaticCapture("Drive Camera", 1);
    
    // Configure the button bindings
    configureBindings();

    // Setup SmartDashboard Auton optionsm
    sc_autonSelect.setDefaultOption("Don't Move", new AutonomousDontMove(m_robotDrive, m_robotIntake));
    sc_autonSelect.addOption("CrossLine", new AutonomousCrossLine(m_robotDrive, m_robotIntake));
    sc_autonSelect.addOption("ShootCrossLine", new AutonomousOneNote(m_robotArm, m_robotDrive, m_robotIntake, m_robotShoot));
    sc_autonSelect.addOption("2 Note Shoot", new AutonomousTwoNote(m_robotDrive, m_robotIntake, m_robotArm, m_robotShoot));

    SmartDashboard.putData("Auton Selection", sc_autonSelect);

    // Configure default commands
    m_robotDrive.setDefaultCommand(drivetrainDefaultCommand);
    m_robotIntake.setDefaultCommand(intakeDefaultCommand);
    m_robotArm.setDefaultCommand(armDefaultCommand);
    m_robotShoot.setDefaultCommand(shootDefaultCommand);
  }

  private void configureBindings() {

    // Intake Wheels: PovUp-Out PovDown-In
    
    new POVButton(m_mechController, OIConstants.kUpDPad)
      .onTrue(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kOut))
      .onFalse(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kOff));     
    new POVButton(m_mechController, OIConstants.kDownDPad)
      .onTrue(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kIn))
      .onFalse(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kOff));
    
    // Arm: Y-Up Position A-Down Position X-Middle Position
    new JoystickButton(m_mechController, Button.kY.value)
      .onTrue(new ArmSetPosCommand(ArmSubsystem.ArmPos.kHome));
    new JoystickButton(m_mechController, Button.kA.value)
      .onTrue(new ArmSetPosCommand(ArmSubsystem.ArmPos.kExtend));
    new JoystickButton(m_mechController, Button.kX.value)
      .onTrue(new ArmSetPosCommand(ArmSubsystem.ArmPos.kMid));
    new JoystickButton(m_mechController, Button.kB.value)
      .onTrue(new ArmSetPosCommand(ArmSubsystem.ArmPos.kFloor));

    new JoystickButton(m_mechController, Button.kLeftBumper.value)
      .toggleOnTrue(new ShootSetSpdCommand(m_robotShoot, ShooterSubsystem.shootDir.kOut));    
    new JoystickButton(m_mechController, Button.kRightBumper.value)
      .onTrue(new ShootSetSpdCommand(m_robotShoot, ShooterSubsystem.shootDir.kIn))
      .onFalse(new ShootSetSpdCommand(m_robotShoot, ShooterSubsystem.shootDir.kOff));
    new JoystickButton(m_mechController, Button.kLeftStick.value)
      .toggleOnTrue(new ShootSetSpdLowCommand(m_robotShoot, ShooterSubsystem.shootDir.kOut));
    
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
      .onTrue(new ClimbSetSpdCommand(m_robotClimb, 0.5))
      .onFalse(new ClimbSetSpdCommand(m_robotClimb, 0));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
      .onTrue(new ClimbSetSpdCommand(m_robotClimb, -0.5))
      .onFalse(new ClimbSetSpdCommand(m_robotClimb, 0));

    new JoystickButton(m_driverController, Button.kA.value)
      .onTrue(new ZeroHeading(m_robotDrive));
    

  }

  public Command getAutonomousCommand() {
    return sc_autonSelect.getSelected();
  }
}
