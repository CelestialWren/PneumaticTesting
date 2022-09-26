// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleboardInfo extends SubsystemBase {
  /** Creates a new Shuffleboard. */
  static ShuffleboardInfo  instance;
  static ShuffleboardTab  pneumaticControl;
  static ShuffleboardTab drivetrainTab;

  
  private  ShuffleboardInfo() {
    pneumaticControl = Shuffleboard.getTab("Pneumatic Control");
    drivetrainTab = Shuffleboard.getTab("Drivetrain");

  }

  public static ShuffleboardInfo getInstance(){
      if(instance == null)
        instance = new ShuffleboardInfo();
      return  instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public  ShuffleboardTab getPneumaticControl() {
      return pneumaticControl;
  }

  public static ShuffleboardTab getDrivetrainTab() {
      return drivetrainTab;
  }
}
