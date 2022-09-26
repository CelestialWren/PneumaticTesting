// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Pneumatics.ExtendIntake;
import frc.robot.commands.Pneumatics.RetractIntake;
import frc.robot.commands.Vision.AlignToTarget;
import frc.robot.commands.Vision.DriveToTarget;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final Pneumatics pneumatics = Pneumatics.getInstance();
  private final Drivetrain mDrivetrain = Drivetrain.getInstance();
  private final Vision vision = Vision.getInstance();
  
  private static final XboxController driverController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    pneumatics.enableCompressor(true);
    configureButtonBindings();
    mDrivetrain.setDefaultCommand(getTeleopDrive());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   JoystickButton retractIntakeButton = new JoystickButton(driverController, XboxController.Button.kA.value);
   JoystickButton extendIntakeButton = new JoystickButton(driverController, XboxController.Button.kB.value);
   JoystickButton alignRobotButton = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
   Trigger DriveToTargetButton = new Trigger(driverController::getRightBumper);


   retractIntakeButton.whenPressed(new RetractIntake());
   extendIntakeButton.whenPressed(new ExtendIntake());
   alignRobotButton.whileActiveOnce(new AlignToTarget());
   DriveToTargetButton.and(new Trigger(vision::hasTargets)).whileActiveOnce(new DriveToTarget());


   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint
      (new SimpleMotorFeedforward(Constants.kS, Constants.kV), Constants.kDriveKinematics, 10);


    TrajectoryConfig trajConfig = 
      new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, 
          Constants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(Constants.kDriveKinematics)
          .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = 
      TrajectoryGenerator.generateTrajectory
        (new Pose2d(0,0, new Rotation2d(0)), 
        List.of(new Translation2d(2,0), new Translation2d(4, -0.5), new Translation2d(6,0), 
                new Translation2d(8,0.5)),
        new Pose2d(10,0, new Rotation2d(0)),
        trajConfig);
       
    RamseteCommand ramseteCommand = 
      new RamseteCommand(
        exampleTrajectory,
        mDrivetrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA),
        Constants.kDriveKinematics,
        mDrivetrain::getWheelSpeeds,
        new PIDController(Constants.kP_VEL_SPARK, 0, 0),
        new PIDController(Constants.kP_VEL_SPARK, 0, 0),
        mDrivetrain::tankDriveVolts,
        mDrivetrain);
      
    mDrivetrain.zeroHeading();  
    mDrivetrain.resetOdometry(exampleTrajectory.getInitialPose());
      
    return ramseteCommand.andThen(()->mDrivetrain.tankDriveVolts(0, 0));
  }

  public Command getTeleopDrive(){

    return new RunCommand(() -> mDrivetrain.arcadeDrive(driverController.getLeftY(), driverController.getRightX()), mDrivetrain);
  }
}
