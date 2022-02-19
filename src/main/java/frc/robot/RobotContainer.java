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
import frc.robot.subsystems.ExampleSubsytem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    // joysticks
    public static final XboxController driverJoystick = new XboxController(
            Constants.JOYSTICK_NUMBER);
    public static final DriveStick leftStick = new DriveStick(
            Constants.LEFT_STICK);
    public static final DriveStick rightStick = new DriveStick(
            Constants.RIGHT_STICK);

    // subsystems
    private final DriveTrain driveTrain;
    private final Shooter shooter;
    private final Intake intake;
    private final ExampleSubsytem subsytem;

    // commands
    //    private final DriveWithJoysticks driveWithJoysticks;
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
        subsytem = new ExampleSubsytem();

        //        driveWithJoysticks = new DriveWithJoysticks(driveTrain);
        //        driveWithJoysticks.addRequirements(driveTrain);
        tankDriveCommand = new TankDriveCommand(driveTrain);

        driveTrain.setDefaultCommand(tankDriveCommand);

        shootBall = new ShootBall(shooter);
        shootBall.addRequirements(shooter);

        autoShoot = new AutoShoot(shooter);
        autoShoot.addRequirements(shooter);

        intakeBall = new IntakeBall(intake);
        intakeBall.addRequirements(intake);
        intake.setDefaultCommand(intakeBall);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        JoystickButton shootButton = new JoystickButton(driverJoystick,
                                                        XboxController.Button.kRightBumper.value);
        shootButton.whileHeld(new ShootBall(shooter));

        JoystickButton toggleButton = new JoystickButton(driverJoystick,
                                                         XboxController.Button.kLeftBumper.value);
        toggleButton.whenPressed(subsytem::togglePiston, subsytem);
    }
}
