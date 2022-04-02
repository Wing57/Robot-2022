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
import frc.robot.commands.ShootBall;
import frc.robot.commands.Auton.AutoShoot;
import frc.robot.commands.Auton.TestCommandGroup;
import frc.robot.commands.Drive.TankDriveCommand;
import frc.robot.commands.Index.IndexBall;
import frc.robot.commands.Index.ReverseIndex;
import frc.robot.commands.Intake.IntakeBall;
import frc.robot.commands.Intake.OuttakeBall;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
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
	public static final Gamepad gamepad = new Gamepad(Constants.XBOXCONTROLLER);
	public static final XboxController DriverStick = new XboxController(
	  Constants.XBOXCONTROLLER2);
	public static final DriveStick leftStick = new DriveStick(Constants.LEFT_JOY);
	public static final DriveStick rightStick = new DriveStick(Constants.RIGHT_JOY);

	// subsystems
	private final DriveTrain driveTrain;
	private final Shooter shooter;
	private final Intake intake;
	private final Climber climber;
	private final Index index;

	// commands
	// private final DriveWithJoysticks driveWithJoysticks;
	private final TankDriveCommand tankDriveCommand;
	public static ShootBall shootBall;
	public static AutoShoot autoShoot;
	public static IntakeBall intakeBall;
	public static OuttakeBall outtakeBall;
	public static TestCommandGroup group;
	public static IndexBall indexBall;
	public static ReverseIndex reverseIndex;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		driveTrain = new DriveTrain();
		shooter = new Shooter();
		intake = new Intake();
		climber = new Climber();
		index = new Index();

		shooter.setDefaultCommand(new RunCommand(() -> shooter.setTurretSpeed(gamepad
		  .getAxisValue(Gamepad.Axis.LeftYAxis)
		  * 0.35), shooter));

		shooter.setReverseShooterSpeed(-gamepad.getAxisValue(Gamepad.Axis.RightTrigger) * 0.30);

		// right drivestick trigger -> shift gears
		tankDriveCommand = new TankDriveCommand(driveTrain, rightStick.getButton(
		  DriveStick.ButtonType.button1));

		driveTrain.setDefaultCommand(tankDriveCommand);

		climber.setDefaultCommand(new RunCommand(() -> climber.setActmotor(gamepad
		  .getAxisValue(Gamepad.Axis.RightYAxis)), climber));

		shootBall = new ShootBall(shooter);

		autoShoot = new AutoShoot(shooter);

		intakeBall = new IntakeBall(intake);

		outtakeBall = new OuttakeBall(intake);

		group = new TestCommandGroup(driveTrain, shooter);

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

		// B -> Outtake

		gamepad.getButton(Gamepad.ButtonType.B).whileHeld(new OuttakeBall(intake), false);

		// X -> Index

		gamepad.getButton(Gamepad.ButtonType.X).whileHeld(new IndexBall(index), false);

		// Y -> Reverse Index

		gamepad.getButton(Gamepad.ButtonType.Y).whileHeld(new ReverseIndex(index), false);

		// LEFT BUMPER -> SHIFT GEARS CRY

		gamepad.getButton(Gamepad.ButtonType.LeftBumper).whileHeld(driveTrain::shiftGears,
		  driveTrain);

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
