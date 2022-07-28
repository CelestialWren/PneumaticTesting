// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

private static Drivetrain instance;

  private static final int mLeftMotorLeaderPort = 4;
  private static final int mLeftMotorFollowerPort = 1;
  private static final int mRightMotorLeaderPort = 3;
  private static final int mRightMotorFollowerPort = 2;

  private CANSparkMax mLeftMotorLeader = new CANSparkMax(mLeftMotorLeaderPort, MotorType.kBrushless);
  private CANSparkMax mLeftMotorFollower = new CANSparkMax(mLeftMotorFollowerPort, MotorType.kBrushless);
  private CANSparkMax mRightMotorLeader = new CANSparkMax(mRightMotorLeaderPort, MotorType.kBrushless);
  private CANSparkMax mRightMotorFollower = new CANSparkMax(mRightMotorFollowerPort, MotorType.kBrushless);

  private AHRS gyro = new AHRS();
  private RelativeEncoder mLeftEncoder = mLeftMotorLeader.getEncoder();
  private RelativeEncoder mRightEncoder = mRightMotorLeader.getEncoder();

  private DifferentialDrive mDrivetrain;

  
  private Rotation2d mRotation;
  private DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(mRotation);

  private double mSpeedMultiplier = 0.5;
  
 public static Drivetrain getInstance(){
  if (instance != null)
    return instance;
  else  
    instance = new Drivetrain();
    return instance;
 }
 
  /** Creates a new Drivetrain. */
  private Drivetrain() {

    mLeftMotorLeader.restoreFactoryDefaults();
    mLeftMotorFollower.restoreFactoryDefaults();
    mRightMotorLeader.restoreFactoryDefaults();
    mRightMotorFollower.restoreFactoryDefaults();

    mLeftMotorLeader.setInverted(true);
    mLeftMotorFollower.setInverted(true);

    mRightMotorFollower.follow(mRightMotorLeader);
    mLeftMotorFollower.follow(mLeftMotorLeader);

    mLeftEncoder.setPositionConversionFactor(Constants.Drivetrain.WHEEL_CIRCUMFRENCE/360);
    mLeftEncoder.setVelocityConversionFactor(Constants.Drivetrain.WHEEL_CIRCUMFRENCE/360);
    mRightEncoder.setPositionConversionFactor(Constants.Drivetrain.WHEEL_CIRCUMFRENCE/360);
    mRightEncoder.setVelocityConversionFactor(Constants.Drivetrain.WHEEL_CIRCUMFRENCE/360);

    mDrivetrain = new DifferentialDrive(mLeftMotorLeader, mRightMotorLeader);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }

  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    mDrivetrain.tankDrive(mSpeedMultiplier * leftSpeed, mSpeedMultiplier * rightSpeed);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    mLeftMotorLeader.setVoltage(leftVolts);
    mRightMotorLeader.setVoltage(rightVolts);
  }

  public void arcadeDrive(double forwardSpeed, double turnSpeed){
    mDrivetrain.arcadeDrive(forwardSpeed, turnSpeed, true);
  }

  public void setBrakeMode(CANSparkMax.IdleMode brakeMode){
    mLeftMotorLeader.setIdleMode(brakeMode);
    mRightMotorLeader.setIdleMode(brakeMode);
    mLeftMotorFollower.setIdleMode(brakeMode);
    mRightMotorFollower.setIdleMode(brakeMode);
  }

//meters
  public double getLeftEncoderDistance(){
    return mLeftEncoder.getPosition();
  }

  //meters
  public double getRightEncoderDistance(){
    return mRightEncoder.getPosition();
  }


  public double getRotation(){
    return mRotation.getDegrees();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(mLeftEncoder.getVelocity(), mRightEncoder.getVelocity());
  }

  public Pose2d getPose(){
    return mOdometry.getPoseMeters();
  }


  public void resetOdometry(){
    gyro.reset();
    mRotation = gyro.getRotation2d();
    mOdometry.update(mRotation, 0, 0);
  }
  public void updateOdometry(){
    mRotation = gyro.getRotation2d();
    mOdometry.update(mRotation, getLeftEncoderDistance(), getRightEncoderDistance());
    
  }

}
