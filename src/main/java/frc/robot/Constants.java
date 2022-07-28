// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final class Drivetrain {
        public static final double WHEEL_CIRCUMFRENCE = 18.8495559215;
        public static final double wheelCircumfrence = 18.8495559215;
        public static final double B = 0;
        public static final double ZETA = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double TRACK_WIDTH = Units.inchesToMeters(23);
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;
           
        }
    
    public static final class Vision{
        // Constants such as camera and target height stored. Change per robot and goal!
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
   public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
         // Angle between horizontal and the camera.
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    public static final double TARGET_PITCH_RADIANS = Units.degreesToRadians(0);
    public static final double DISTANCE_TO_ALIGN = Units.feetToMeters(1.5);
    public static final double DISTANCE_TOLERANCE = Units.inchesToMeters(4);
    public static final double ANGLE_TOLERANCE = 1;

    }

    }
