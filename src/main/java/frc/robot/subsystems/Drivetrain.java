// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.ml.Ml;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SerialPort;

public class Drivetrain extends SubsystemBase {

  private static final int mLeftMotorLeaderPort = 4;
  private static final int mLeftMotorFollowerPort = 1;
  private static final int mRightMotorLeaderPort = 3;
  private static final int mRightMotorFollowerPort = 2;

  private CANSparkMax mLeftMotorLeader = new CANSparkMax(mLeftMotorLeaderPort, MotorType.kBrushless);
  private CANSparkMax mLeftMotorFollower = new CANSparkMax(mLeftMotorFollowerPort, MotorType.kBrushless);
  private CANSparkMax mRightMotorLeader = new CANSparkMax(mRightMotorLeaderPort, MotorType.kBrushless);
  private CANSparkMax mRightMotorFollower = new CANSparkMax(mRightMotorFollowerPort, MotorType.kBrushless);

  private RelativeEncoder mLeftEncoder = mLeftMotorLeader.getEncoder();
  private RelativeEncoder mRightEncoder = mRightMotorLeader.getEncoder();
  
  private SparkMaxPIDController mLeftPIDController = mLeftMotorLeader.getPIDController();
  private SparkMaxPIDController mRightPIDController = mRightMotorLeader.getPIDController();


  private final AHRS mGyro = new AHRS(SerialPort.Port.kUSB);  
  private final DifferentialDriveOdometry mOdometry;

  private DifferentialDrive mDrivetrain;
  private double mSpeedMultiplier = 0.5;
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {

    mLeftMotorLeader.restoreFactoryDefaults();
    mLeftMotorFollower.restoreFactoryDefaults();
    mRightMotorLeader.restoreFactoryDefaults();
    mRightMotorFollower.restoreFactoryDefaults();

    mLeftMotorLeader.setInverted(true);
    mLeftMotorFollower.setInverted(true);

    mRightMotorFollower.follow(mRightMotorLeader);
    mLeftMotorFollower.follow(mLeftMotorLeader);

    mDrivetrain = new DifferentialDrive(mLeftMotorLeader, mRightMotorLeader);

    // 6 inch diameter wheels
    // 0.09337068 gear ratio
    // 60 second conversion
    mLeftEncoder.setVelocityConversionFactor(Math.PI*Units.inchesToMeters(6)*0.09337068/60);
    mLeftEncoder.setPositionConversionFactor(Math.PI*Units.inchesToMeters(6)*0.09337068);
    mRightEncoder.setVelocityConversionFactor(Math.PI*Units.inchesToMeters(6)*0.09337068/60);
    mRightEncoder.setPositionConversionFactor(Math.PI*Units.inchesToMeters(6)*0.09337068);

    mLeftPIDController.setP(Constants.kP_VEL);
    mLeftPIDController.setD(Constants.kD_VEL);
    mRightPIDController.setP(Constants.kP_VEL);
    mRightPIDController.setD(Constants.kD_VEL);


    resetEncoders();
    mGyro.reset();
    mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d());

    Shuffleboard.getTab("Gyro").add(mGyro);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mOdometry.update(mGyro.getRotation2d().unaryMinus(), mLeftEncoder.getPosition(), mRightEncoder.getPosition());
  }

  public Pose2d getPose()
  {
    return mOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    return new DifferentialDriveWheelSpeeds(mLeftEncoder.getVelocity(), mRightEncoder.getVelocity());
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    mOdometry.resetPosition(pose, mGyro.getRotation2d().unaryMinus());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts)
  {
    mLeftMotorLeader.setVoltage(leftVolts*12); // Convert this from percent of battery to volts by multiply by 12
    mRightMotorLeader.setVoltage(rightVolts*12); // Convert this from percent of battery to volts by multiply by 12
  }

  public void resetEncoders()
  {
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
  }

  public RelativeEncoder getLeftEncoder()
  {
    return mLeftEncoder;
  }

  public RelativeEncoder getRightEncoder()
  {
    return mRightEncoder;
  }

  public void zeroHeading()
  {
    mGyro.reset();
  }

  public double getHeading()
  {
    return -mGyro.getRotation2d().getDegrees();
  }

  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    mDrivetrain.tankDrive(mSpeedMultiplier * leftSpeed, mSpeedMultiplier * rightSpeed);
  }

  
  public SparkMaxPIDController getLeftPidController(){
    return mLeftPIDController;
  }
    public SparkMaxPIDController getRightPidController(){
    return mRightPIDController;
    }
  }

