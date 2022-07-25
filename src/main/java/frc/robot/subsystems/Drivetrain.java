// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private static final int mLeftMotorLeaderPort = 4;
  private static final int mLeftMotorFollowerPort = 1;
  private static final int mRightMotorLeaderPort = 3;
  private static final int mRightMotorFollowerPort = 2;

  private CANSparkMax mLeftMotorLeader = new CANSparkMax(mLeftMotorLeaderPort, MotorType.kBrushless);
  private CANSparkMax mLeftMotorFollower = new CANSparkMax(mLeftMotorFollowerPort, MotorType.kBrushless);
  private CANSparkMax mRightMotorLeader = new CANSparkMax(mRightMotorLeaderPort, MotorType.kBrushless);
  private CANSparkMax mRightMotorFollower = new CANSparkMax(mRightMotorFollowerPort, MotorType.kBrushless);

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


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    mDrivetrain.tankDrive(mSpeedMultiplier * leftSpeed, mSpeedMultiplier * rightSpeed);
  }

}
