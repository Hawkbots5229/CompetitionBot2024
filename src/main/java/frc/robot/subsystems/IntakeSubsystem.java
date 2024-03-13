// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  public enum intakeDir {kIn, kOut, kOff};

  private final CANSparkMax m_left =
    new CANSparkMax(IntakeConstants.kLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_right =
    new CANSparkMax(IntakeConstants.kRightMotorPort, MotorType.kBrushless);

  private final RelativeEncoder e_LeftEncoder = m_left.getEncoder();
  private final RelativeEncoder e_RightEncoder = m_right.getEncoder();
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    m_left.restoreFactoryDefaults();
    m_left.setInverted(IntakeConstants.kLeftMotorIntverted);
    m_left.setIdleMode(IntakeConstants.kIdleMode);
    m_left.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
    m_left.setOpenLoopRampRate(IntakeConstants.kOpenLoopRampRate);
    m_left.enableVoltageCompensation(IntakeConstants.maxVoltage);
  
    m_right.restoreFactoryDefaults();
    m_right.setInverted(IntakeConstants.kRightMotorInverted);
    m_right.setIdleMode(IntakeConstants.kIdleMode);
    m_right.setSecondaryCurrentLimit(IntakeConstants.kCurrentLimit);
    m_right.setOpenLoopRampRate(IntakeConstants.kOpenLoopRampRate);
    m_right.enableVoltageCompensation(IntakeConstants.maxVoltage);
  }

  /** Sets the desired speed of the left and right intake motors.
   * 
   * @return Void
   * @param output The speed to set. Value should be between -1.0 and 1.0.
   * @implNote com.revrobotics.CANSparkMax.set()
   * 
   */
  public void setTargetOutput(double output) {
    m_left.set(output);
    m_right.set(output);
  }

  /** Spins intake to grap items.
   * 
   * @return Void
   * @param None
   * @implNote setTargetOutput()
   * @implNote IntakeConstants.kMaxOutput
   * 
   */
  public void wheelsIn() {
    setTargetOutput(IntakeConstants.kMaxOutput);
  }
  
  /** Spins intake to eject items.
   * 
   * @return Void
   * @param None
   * @implNote setTargetOutput()
   * @implNote IntakeConstants.kMaxOutput
   * 
   */
  public void wheelsOut() {
    setTargetOutput(-IntakeConstants.kMaxOutput);
  }

  /** Calculates the average motor velocities.
   * 
   * @return Velocity of intake motors (rev/sec)
   * @param None
   * @implNote com.revrobotics.RelativeEncoder.getVelocity()
   * 
   */
  public double getIntakeVel() {
    return (e_LeftEncoder.getVelocity() + e_RightEncoder.getVelocity())/2;
  }

  /** Stops the left and right motors.
   * 
   * @return Void
   * @param None
   * @implNote com.revrobotics.CANSparkMax.stopMotor()
   * 
   */
  public void stopMotors() {
    m_left.stopMotor();
    m_right.stopMotor();
  }

  /** Sets the left and right intake encoders to a position of 0. 
   * 
   * @return Void
   * @param None
   * @implNote com.revrobotics.RelativeEncoder.setPosition()
   * 
   */
  public void resetEncoders() {

    e_LeftEncoder.setPosition(0);
    e_RightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Velocity", getIntakeVel());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}