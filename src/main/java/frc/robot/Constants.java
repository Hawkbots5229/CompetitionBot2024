// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.library.SwerveData;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final String kCanBus = "CANivore";

    public static final int kFrontLeftDriveMotorPort = 42;
    public static final int kRearLeftDriveMotorPort = 40;
    public static final int kFrontRightDriveMotorPort = 43;
    public static final int kRearRightDriveMotorPort = 41;

    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kRearLeftDriveMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    public static final int kFrontLeftTurningMotorPort = 32;
    public static final int kRearLeftTurningMotorPort = 30;
    public static final int kFrontRightTurningMotorPort = 33;
    public static final int kRearRightTurningMotorPort = 31;

    public static final boolean kFrontLeftTurningMotorReversed = false;
    public static final boolean kRearLeftTurningMotorReversed = false;
    public static final boolean kFrontRightTurningMotorReversed = false;
    public static final boolean kRearRightTurningMotorReversed = false;

    public static final int kFrontLeftTurningEncoderPorts = 52;
    public static final int kRearLeftTurningEncoderPorts = 50;
    public static final int kFrontRightTurningEncoderPorts = 53;
    public static final int kRearRightTurningEncoderPorts = 51;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.65405;
    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.43815;
    
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final NeutralModeValue driveMode = NeutralModeValue.Coast;
    public static final NeutralModeValue turnMode = NeutralModeValue.Brake;
    public static final double maxVoltage = 12.0;
    public static final double maxSpeed = 4.36; // m/s
    public static final double maxAngularSpeed = Math.PI*4; // 1/2 Rotation/Sec
    public static final double rotKp = 0.01;
    public static final double rotKi = 0.02;
    public static final double rotKd = 0.0;
    public static final double rotToleranceDeg = 1;
    public static final double rotToleranceVel = 10; // Deg/sec
    public static final double rotMaxOutput  = 0.5;
    public static final double stickDeadband = 0.1;
    public static final double speedScale = 0.6;
    public static final double rotationScale = 0.25;
    public static final double MetersPerInch = 1/39.37008;
    public static final int kCurrentLimit = 40;
    public static final double kOpenLoopRampRate = 0;

    public static final boolean kGyroReversed = true;

    public static final boolean kuseAbsEnc = true;

    public static final SwerveData SDFrontLeft = new SwerveData("FL", 
    kFrontLeftDriveMotorPort, 
    kFrontLeftDriveMotorReversed, 
    kFrontLeftTurningMotorPort, 
    kFrontLeftTurningMotorReversed, 
    kFrontLeftTurningEncoderPorts,
    kFrontLeftTurningEncoderReversed,
    -0.398, //-0.6
    kuseAbsEnc);

    public static final SwerveData SDFrontRight = new SwerveData("FR", 
    kFrontRightDriveMotorPort, 
    kFrontRightDriveMotorReversed, 
    kFrontRightTurningMotorPort, 
    kFrontRightTurningMotorReversed, 
    kFrontRightTurningEncoderPorts,
    kFrontRightTurningEncoderReversed,
    -0.970, //-0.029
    kuseAbsEnc);

  public static final SwerveData SDRearLeft = new SwerveData("RL", 
    kRearLeftDriveMotorPort, 
    kRearLeftDriveMotorReversed, 
    kRearLeftTurningMotorPort, 
    kRearLeftTurningMotorReversed, 
    kRearLeftTurningEncoderPorts,
    kRearLeftTurningEncoderReversed,
    -0.370, //-0.627
    kuseAbsEnc);

  public static final SwerveData SDRearRight = new SwerveData("RR", 
    kRearRightDriveMotorPort, 
    kRearRightDriveMotorReversed, 
    kRearRightTurningMotorPort, 
    kRearRightTurningMotorReversed, 
    kRearRightTurningEncoderPorts, 
    kRearRightTurningEncoderReversed,
    -0.895, //-0.122
    kuseAbsEnc);
  }

  public static final class SwerveConstants {

    public static final double steerKp = 0.55;
    public static final double steerKi = 0.15;
    public static final double steerKd = 0;

    public static final double steerMax_RadPS = Math.PI;
    public static final double steerMax_RadPSSq = Math.pow(steerMax_RadPS,2);
    public static final double steerRatio = 396.0/35.0; // Per Westcoast products website
    public static final double steer_RevPRad = steerRatio/(2.0*Math.PI); 
    
    
    public static final double driveDistanceCntsPMeter = 49907;
    public static final double driveRawVelocityToMPS = 4990.68;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechControllerPort = 1;

    public static final int kUpDPad = 180;
    public static final int kDownDPad = 0;
  }

  public static class IntakeConstants{

    public static final int kRightMotorPort = 26;

    public static final boolean kRightMotorInverted = true;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final double maxVoltage = 12.0;
    public static final int kCurrentLimit = 30;
    public static final double kOpenLoopRampRate = 0;
    public static final double kMaxOutput = 1.0;
        
    public static final double kGearBoxRatio = 1/7.0;
    public static final double kEncoderRpmToWheelRpm = kGearBoxRatio;

    public static final double kMaxVel = 1; //Wheel RPM
  }

  public static class ArmPivotConstants {

    public static final int kRightMotorPort = 20;
    public static final int kLeftMotorPort = 21;

    public static final boolean kRightMotorInverted = true;
    public static final boolean kLeftMotorInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final double maxVoltage = 12.0;
    public static final int kCurrentLimit = 40;
    public static final double kOpenLoopRampRate = 10;
   
    public static final double kGearBoxRatio = (1/100.0);
    public static final double kSprocketRatio = (22.0/60.0); 
    public static final double kEncoderRevToArmRads = kGearBoxRatio*kSprocketRatio*2.0*Math.PI;

    public static final double kPPos = 0.5;
    public static final double kIPos= 0.01;
    public static final double kDPos = 0;
    public static final double kPosErrTolerance = 1;

    public static final double kMaxVel = Math.PI/4.0; //RadPerSec
    public static final double kMaxAcc = Math.pow(kMaxVel,2);; //RadPerSecSqrd

    public static final double kHomeLoc = 0.0;
    public static final double kExtendLoc = 78.0;
    public static final double kMidLoc = 37.0;
  }

  public static class ClimbConstants {
    public static final int kRightMotorPort = 22;
    public static final int kLeftMotorPort = 23;

    public static final boolean kRightMotorInverted = true;
    public static final boolean kLeftMotorInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final double maxVoltage = 12.0;
    public static final int kCurrentLimit = 40;
  }

    public static class ShooterConstants {
    public static final int kRightMotorPort = 24;
    public static final int kLeftMotorPort = 25;

    public static final boolean kRightMotorInverted = true;
    public static final boolean kLeftMotorInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kCoast;
    public static final double maxVoltage = 12.0;
    public static final int kCurrentLimit = 40;
  }
}
