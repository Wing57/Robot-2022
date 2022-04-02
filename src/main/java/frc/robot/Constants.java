// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	// *****************************************
	// ********** DRIVE TRAIN ******************
	// *****************************************
	public static final int RIGHT_MOTOR_1 = 3;
	public static final int RIGHT_MOTOR_2 = 2;
	public static final int RIGHT_MOTOR_3 = 1;
	public static final int LEFT_MOTOR_1 = 4;
	public static final int LEFT_MOTOR_2 = 5;
	public static final int LEFT_MOTOR_3 = 6;

	public static final int SHIFTER_FORWARD_CHANNEL = 8;
	public static final int SHIFTER_REVERSE_CHANNEL = 9;

	// Drivetrain Speed
	public static final double DRIVETRAINSPEED = 1;

	// Duration of DRIVEFORWARD Command
	public static final double DRIVE_FORWARD_TIME = 3.0;

	// Speed During AUTO
	public static final double AUTONOMOUS_SPEED = 0.4;

	// *****************************************
	// ************** SHOOTER ******************
	// *****************************************
	public static final int SHOOTER_MOTOR_1 = 8;
	public static final int SHOOTER_MOTOR_2 = 7;
	public static final int TURRET_MOTOR = 9;

	public static final double AUTO_SHOOT_TIME = 2.0;
	public static final double LL_SHOT_HEIGHT = 5;
	public static final double SHOOT_SPEED = 0.30;
	// good speed value is 0.38
	public static final double BACKSPIN_SPEED = 0.38;
	// good speed value is 0.45
	public static final int RIGHTTURRET_LIMITSWITCH_PORT = 9;
	public static final int LEFTTURRET_LIMITSWITCH_PORT = 8;

	// *****************************************
	// ************** INTAKE *******************
	// *****************************************
	public static final int INTAKE_MOTOR_1 = 10;
	public static final int MODULE_NUMBER = 2;

	public static final double INTAKE_SPEED = 0.75;

	// *****************************************
	// *********** INTAKE EXTEND ***************
	// *****************************************
	public static final int INTAKE_PISTON_FORWARD_CHANNEL = 0;
	public static final int INTAKE_PISTON_REVERSE_CHANNEL = 1;

	// *****************************************
	// ************** JOYSTICKS ****************
	// *****************************************
	public static final int LEFT_JOY = 0;
	public static final int RIGHT_JOY = 1;
	public static final int XBOXCONTROLLER = 2;
	public static final int XBOXCONTROLLER2 = 3;

	// *****************************************
	// ************** Climber ****************
	// *****************************************
	public static final int ACTUATOR_MOTOR = 11;
    public static final int HOOK_MOTOR = 13;

	// *****************************************
	// ************** STORAGE ****************
	// *****************************************

	public static final int INDEX_MOTOR = 12;
	public static final double INDEX_SPEED = 1;


}
