// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int BMOTOR_LEFT = 3;
    public static final Translation2d BMOTOR_LEFT_POS = new Translation2d(-1, -1);
    public static final int BMOTOR_RIGHT = 4;
    public static final Translation2d BMOTOR_RIGHT_POS = new Translation2d(1, -1);
    public static final int FMOTOR_LEFT = 1;
    public static final Translation2d FMOTOR_LEFT_POS = new Translation2d(-1, 1);
    public static final int FMOTOR_RIGHT = 2;
    public static final Translation2d FMOTOR_RIGHT_POS = new Translation2d(1, 1);

    public static final Pose2d DEFUALT_POS = new Pose2d(0,0, new Rotation2d(0));

    public static double DRIVE_SPEED = 1.0;
    public static double MAX_SPEED = 1.0;
    public static double ROTATE_SPEED = 1.0; 

    public static final double TALON_UNITS_PER_ROTATION = 2048;
    public static final double SHAFT_TO_WHEEL_GEAR_RATIO = (8.46/1)/1;
    public static final double WHEEL_RADIUS = 3 * 0.0254;
    public static final double WHEEL_CIRCUMFERENCE = 2 * WHEEL_RADIUS * Math.PI;

    public static double FF_KS = 0.0;
    public static double FF_KV = 0.0;
    public static double FF_KA = 0.0;
     

}
