// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToTarget extends PIDCommand {
  /** Creates a new AlignToTarget. */
  private static Drivetrain drivetrain = Drivetrain.getInstance();
  private static Vision vision = Vision.getInstance();
  public AlignToTarget() {
    super(
        // The controller that the command will use
        new PIDController(Constants.Drivetrain.KP, Constants.Drivetrain.KI, Constants.Drivetrain.KD),
        // This should return the measurement
        () ->(-1*vision.getAngle()),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          drivetrain.arcadeDrive(0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
