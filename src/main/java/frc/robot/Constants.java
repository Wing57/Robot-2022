// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Talon Can ID's (NOT REAL VALUES YET) 
    public static final int RIGHT_MOTOR_1 = 0;
    public static final int RIGHT_MOTOR_2 = 1;
    public static final int RIGHT_MOTOR_3 = 2;
    public static final int LEFT_MOTOR_1 = 3;
    public static final int LEFT_MOTOR_2 = 4;
    public static final int lEFT_MOTOR_3 = 5;
    public static final int SHOOTER1 = 6;
    public static final int INTAKE1 = 7;

    // XboxController Axis values
    public static final int XBOX_LEFT_Y_AXIS = 1;
    public static final int XBOX_LEFT_X_AXIS = 0;
    public static final int RIGHT_TRIGGER = 3;

    // Drivetrain Speed
    public static final double DRIVETRAINSPEED = 0.75;

    // Duration of DRIVEFORWARD Command
    public static final double DRIVE_FORWARD_TIME = 3.0;

    // Speed During AUTO
    public static final double AUTONOMOUS_SPEED = 0.4;

    // XboxController Port Number
    public static final int JOYSTICK_NUMBER = 0;

    // Shoot Speed
    public static final double SHOOT_SPEED = 1;

    // Intake Speed
    public static final double INTAKE_SPEED = 0.75;

    // Camera Resolutions
    public static final int CAMERA_RES_X = 320;
    public static final int CAMERA_RES_Y = 240;

    
    public static final double AUTO_SHOOT_TIME = 2.0;
    
    
}
