// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private static final int mLeftMotorLeaderPort = 3;
  private static final int mRightMotorLeaderPort = 4;

  private CANSparkMax mLeftMotorLeader = new CANSparkMax(mLeftMotorLeaderPort, MotorType.kBrushless);
  private CANSparkMax mRightMotorLeader = new CANSparkMax(mRightMotorLeaderPort, MotorType.kBrushless);

  private DifferentialDrive mDrivetrain;
  private double mSpeedMultiplier = 0.5;
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {

    mLeftMotorLeader.restoreFactoryDefaults();
    mRightMotorLeader.restoreFactoryDefaults();

    mLeftMotorLeader.setInverted(true);

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
  public void arcadeDrive(double xSpeed, double zRotation){
    mDrivetrain.arcadeDrive(mSpeedMultiplier * xSpeed, mSpeedMultiplier * zRotation);
  }

}
