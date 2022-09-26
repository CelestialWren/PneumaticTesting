// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowPath extends RamseteCommand {
  private static Drivetrain mDrivetrain = Drivetrain.getInstance();
  /** Creates a new FollowPath. */
  public FollowPath(Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    super( trajectory,
     mDrivetrain::getPose,
     new RamseteController(Constants.Drivetrain.B, Constants.Drivetrain.ZETA),
     new SimpleMotorFeedforward(Constants.Drivetrain.KS, Constants.Drivetrain.KV),
     new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH),
     mDrivetrain::getWheelSpeeds,
     new PIDController(Constants.Drivetrain.KP, Constants.Drivetrain.KI, Constants.Drivetrain.KD),
     new PIDController(Constants.Drivetrain.KP, Constants.Drivetrain.KI, Constants.Drivetrain.KD),
    mDrivetrain::tankDriveVolts,
    mDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivetrain.resetOdometry();
    mDrivetrain.setBrakeMode(IdleMode.kBrake);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.resetOdometry();
    mDrivetrain.tankDrive(0, 0);
    mDrivetrain.setBrakeMode(IdleMode.kCoast);
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
