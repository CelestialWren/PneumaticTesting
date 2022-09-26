// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Feedforward constants
    public static final double kS = 0.015;
    public static final double kV = 0.21;
    public static final double kA = 0;

    // SparkMax PID Values for Velocity
    public static final double kP_VEL_SPARK = 0.0000013185;
    public static final double kD_VEL_SPARK = 0.0;
    public static final double kP_VEL = 0.12;
    public static final double kD_VEL = 0.0;

    //Kinematics
    public static final double kTrackWidthMeters = Units.inchesToMeters(27.0); //???
    public static final DifferentialDriveKinematics kDriveKinematics = 
        new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.75;

    //Good guesses
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;


}
