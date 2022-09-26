// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DriveToTarget extends CommandBase {
  /** Creates a new DriveToTarget. */
  private static Drivetrain drivetrain = Drivetrain.getInstance();
  private static Vision vision = Vision.getInstance();
  private PIDController angleController = new PIDController(Constants.Drivetrain.KP, Constants.Drivetrain.KI, Constants.Drivetrain.KD);
  private PIDController distanceController = new PIDController(Constants.Drivetrain.KP, Constants.Drivetrain.KI, Constants.Drivetrain.KD);
  private double forwardSpeed;
  private double turnSpeed;
  public DriveToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    angleController.setSetpoint(0);
    angleController.setTolerance(Constants.Vision.ANGLE_TOLERANCE);
    distanceController.setSetpoint(Constants.Vision.DISTANCE_TO_ALIGN);
    distanceController.setTolerance(Constants.Vision.DISTANCE_TOLERANCE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnSpeed = angleController.calculate(vision.getAngle());
    forwardSpeed = distanceController.calculate(vision.getDistance());
    drivetrain.arcadeDrive(forwardSpeed, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distanceController.atSetpoint();
  }
}
