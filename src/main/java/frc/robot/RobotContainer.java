// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.rambots4571.rampage.joystick.DriveStick;
import com.rambots4571.rampage.joystick.Gamepad;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.commands.AutoShoot;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.OuttakeBall;
import frc.robot.commands.ShootBall;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.TestCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeExtend;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

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
	public static final Gamepad gamepad = new Gamepad(Constants.XBOXCONTROLLER);
	public static final DriveStick leftStick = new DriveStick(Constants.LEFT_JOY);
	public static final DriveStick rightStick = new DriveStick(Constants.RIGHT_JOY);

	// subsystems
	private final DriveTrain driveTrain;
	private final Shooter shooter;
	private final Intake intake;
	private final IntakeExtend intakeExtend;
	private final Turret turret;

	// commands
	// private final DriveWithJoysticks driveWithJoysticks;
	private final TankDriveCommand tankDriveCommand;
	public static ShootBall shootBall;
	public static AutoShoot autoShoot;
	public static IntakeBall intakeBall;
	private final TestCommandGroup group;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		driveTrain = new DriveTrain();
		shooter = new Shooter();
		intake = new Intake();
		intakeExtend = new IntakeExtend();
		turret = new Turret();

		turret.setDefaultCommand(new RunCommand(() -> turret.setTurretMotor(gamepad
		  .getAxisValue(Gamepad.Axis.LeftYAxis)), turret));

		group = new TestCommandGroup(driveTrain, shooter);

		// right drivestick trigger -> shift gears
		tankDriveCommand = new TankDriveCommand(driveTrain, rightStick.getButton(
		  DriveStick.ButtonType.button1));

		driveTrain.setDefaultCommand(tankDriveCommand);

		shootBall = new ShootBall(shooter);

		autoShoot = new AutoShoot(shooter);

		intakeBall = new IntakeBall(intake);

		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
	 * then passing it to a
	 * {@link edu.wpi.first.wpililibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// right bumper -> shoot ball
		gamepad.getButton(Gamepad.ButtonType.RightBumper).whileHeld(new ShootBall(shooter),
		  false);

		// A -> intake ball

		gamepad.getButton(Gamepad.ButtonType.A).whileHeld(new IntakeBall(intake), false);

		// B -> outtake ball

		gamepad.getButton(Gamepad.ButtonType.B).whileHeld(new OuttakeBall(intake), false);

		// left bumper -> toggle piston

		gamepad.getButton(Gamepad.ButtonType.LeftBumper).whenPressed(
		  intakeExtend::togglePiston, intakeExtend);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */

	public Command getAutonomousCommand() {
		return group;
	}
}
