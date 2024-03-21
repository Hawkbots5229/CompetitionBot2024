// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  public enum shootDir {kIn, kOut, kOff};

  private final CANSparkMax m_right =
    new CANSparkMax(ShooterConstants.kRightMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_left =
    new CANSparkMax(ShooterConstants.kLeftMotorPort, MotorType.kBrushless);

  private final RelativeEncoder m_rightEncoder = m_right.getEncoder();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    m_right.restoreFactoryDefaults();

    m_right.setInverted(ShooterConstants.kLeftMotorInverted);
    m_right.setIdleMode(ShooterConstants.kIdleMode);
    m_right.enableVoltageCompensation(ShooterConstants.maxVoltage);
    m_right.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);

    m_left.restoreFactoryDefaults();

    m_left.setInverted(ShooterConstants.kLeftMotorInverted);
    m_left.setIdleMode(ShooterConstants.kIdleMode);
    m_left.enableVoltageCompensation(ShooterConstants.maxVoltage);
    m_left.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
  }

  /** Sets the desired speed of the shooter motors.
   * 
   * @return Void
   * @param output The speed to set. Value should be between -1.0 and 1.0.
   * @implNote com.revrobotics.CANSparkMax.set()
   * 
   */
  public void setTargetOutput(double upperOutput, double lowerOutput) {
    m_right.set(lowerOutput);
    m_left.set(upperOutput);
  }

  /** Spins shooter to grab items.
   * 
   * @return Void
   * @param None
   * @implNote setTargetOutput()
   * @implNote ShooterConstants.kMaxUpperOutput
   * @implNote ShooterConstants.kMaxLowerOutput
   * 
   */
  public void wheelsIn() {
    setTargetOutput(ShooterConstants.kMaxUpperOutput, ShooterConstants.kMaxLowerOutput);
  }
  
  /** Spins shooter to eject items.
   * 
   * @return Void
   * @param None
   * @implNote setTargetOutput()
   * @implNote ShooterConstants.kMaxUpperOutput
   * @implNote ShooterConstants.kMaxLowerOutput
   * 
   */
  public void wheelsOut() {
    setTargetOutput(-ShooterConstants.kMaxUpperOutput, -ShooterConstants.kMaxLowerOutput);
  }

/** Gets the current angle of the shooter.
 * 
 * @return Current arm angle (Radians)
 * @param None
 * @implNote com.revrobotics.RelativeEncoder.getPosition()
 */
  public double getAngle() {
    return m_rightEncoder.getPosition() * 2*Math.PI;
  }

/** Gets the current velocity of the shooter.
 * 
 * @return Current arm velocity (Radians/Sec)
 * @param None
 * @implNote com.revrobotics.RelativeEncoder.getVelocity()
 */
  public double getVelocity() {
    return m_rightEncoder.getVelocity() * 2*Math.PI;
  }

  /** Stops the climber motor.
   * 
   * @return Void
   * @param None
   * @implNote com.revrobotics.CANSparkMax.stopMotor()
   * 
   */
  public void stopMotors() {

    m_right.stopMotor();
    m_left.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
