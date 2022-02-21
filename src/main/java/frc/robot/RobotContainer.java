// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.rambots4571.rampage.joystick.DriveStick;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.AutoShoot;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShootBall;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeExtend;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...

	// joysticks
	public static final XboxController gamepad = new XboxController(
	  Constants.XBOXCONTROLLER);
	public static final DriveStick leftStick = new DriveStick(Constants.LEFT_JOY);
	public static final DriveStick rightStick = new DriveStick(Constants.RIGHT_JOY);

	// subsystems
	private final DriveTrain driveTrain;
	private final Shooter shooter;
	private final Intake intake;
	private final IntakeExtend intakeExtend;

	// commands
	// private final DriveWithJoysticks driveWithJoysticks;
	private final TankDriveCommand tankDriveCommand;
	private final ShootBall shootBall;
	private final AutoShoot autoShoot;
	private final IntakeBall intakeBall;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		driveTrain = new DriveTrain();
		shooter = new Shooter();
		intake = new Intake();
		intakeExtend = new IntakeExtend();

		// driveWithJoysticks = new DriveWithJoysticks(driveTrain);
		// driveTrain.setDefaultCommand(driveWithJoysticks);
		tankDriveCommand = new TankDriveCommand(driveTrain);

		driveTrain.setDefaultCommand(tankDriveCommand);

		shootBall = new ShootBall(shooter);

		autoShoot = new AutoShoot(shooter);

		intakeBall = new IntakeBall(intake);
		intake.setDefaultCommand(intakeBall);

		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
	 * then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		JoystickButton shootButton = new JoystickButton(gamepad,
		  XboxController.Button.kRightBumper.value);
		shootButton.whileHeld(new ShootBall(shooter));

		JoystickButton intakeButton = new JoystickButton(gamepad,
		  XboxController.Button.kA.value);
		intakeButton.whileHeld(new IntakeBall(intake));

		JoystickButton toggleButton = new JoystickButton(gamepad,
		  XboxController.Button.kLeftBumper.value);
		toggleButton.whenPressed(intakeExtend::togglePiston, intakeExtend);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */

	// public Command getAutonomousCommand() {
	// // An ExampleCommand will run in autonomous
	// }
}
