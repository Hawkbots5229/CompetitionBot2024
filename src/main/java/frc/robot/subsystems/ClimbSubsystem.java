// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

  private final CANSparkMax m_right =
    new CANSparkMax(ClimbConstants.kRightMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_left =
    new CANSparkMax(ClimbConstants.kLeftMotorPort, MotorType.kBrushless);

  private final RelativeEncoder m_rightEncoder = m_right.getEncoder();
  private final RelativeEncoder m_leftEncoder = m_left.getEncoder();

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {

    m_right.restoreFactoryDefaults();

    m_right.setInverted(ClimbConstants.kLeftMotorInverted);
    m_right.setIdleMode(ClimbConstants.kIdleMode);
    m_right.enableVoltageCompensation(ClimbConstants.maxVoltage);
    m_right.setSmartCurrentLimit(ClimbConstants.kCurrentLimit);

    m_left.restoreFactoryDefaults();

    m_left.setInverted(ClimbConstants.kLeftMotorInverted);
    m_left.setIdleMode(ClimbConstants.kIdleMode);
    m_left.enableVoltageCompensation(ClimbConstants.maxVoltage);
    m_left.setSmartCurrentLimit(ClimbConstants.kCurrentLimit);
    

    resetEncoders();
  }

    /** Sets the climb encoders to a position of 0. 
   * 
   * @return Void
   * @param None
   * @implNote com.revrobotics.RelativeEncoder.setPosition()
   * 
   */
  public void resetEncoders() {

    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);
  }

    /** Gets the current motor angle.
   * 
   * @return Current motor angle (radians)
   * @param None
   * @implNote com.revrobotics.RelativeEncoder.getPosition()
   */
  public double getAngle() {
    return (m_rightEncoder.getPosition() + m_leftEncoder.getPosition())*Math.PI;
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

  /** Sets the desired speed of the left and right climb motors.
   * 
   * @return Void
   * @param output The speed to set. Value should be between -1.0 and 1.0.
   * @implNote com.revrobotics.CANSparkMax.set()
   * 
   */
  public void setTargetOutput(double output) {
    m_right.set(output);
    m_left.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberAngle", Math.toDegrees(getAngle()));

  }
}
