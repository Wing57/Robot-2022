// Copyright (c) Darius and other Darius contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file and DARI lisence in the root directory of this
// project.

package frc.robot;

import com.rambots4571.rampage.command.RunEndCommand;
import com.rambots4571.rampage.joystick.Controller;
import com.rambots4571.rampage.joystick.Gamepad;
import com.rambots4571.rampage.joystick.Gamepad.Button;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Ctake;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Shooters;
import frc.robot.commands.auton.AutoShoot;
import frc.robot.commands.auton.TestCommandGroup;
import frc.robot.commands.auton.TurnCommand;
import frc.robot.commands.drive.FaceHub;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.shooter.ShootBall;
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
  public static final Controller<Gamepad.Button, Gamepad.Axis> gamepad = Gamepad.make(
    Constants.XBOXCONTROLLER);
  public static final Controller<Gamepad.Button, Gamepad.Axis> driveController = Gamepad
    .make(Constants.XBOXCONTROLLER2);

  // subsystems
  private final DriveTrain driveTrain;
  private final Shooter shooter;
  private final Intake intake;
  private final Index index;

  // commands
  private final TankDriveCommand tankDriveCommand;
  public static FaceHub faceHub;

  public static ShootBall shootBall;

  public static TurnCommand turnCommand;
  public static AutoShoot autoShoot;
  public static TestCommandGroup group;

  /**
   * The container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    driveTrain = new DriveTrain();
    shooter = new Shooter();
    intake = new Intake();
    index = new Index();

    // (driveController) X -> SHIFT GEARS
    tankDriveCommand = new TankDriveCommand(driveTrain, driveController.getButton(
      Gamepad.Button.X));

    driveTrain.setDefaultCommand(tankDriveCommand);

    faceHub = new FaceHub(driveTrain);

    shootBall = new ShootBall(shooter);

    autoShoot = new AutoShoot(shooter);

    group = new TestCommandGroup(driveTrain, autoShoot, turnCommand);

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

    gamepad.getButton(Gamepad.Button.RightBumper).whileHeld(new RunEndCommand(() -> {
      shooter.setShooterSpeed(Shooters.SHOOT_SPEED);
      shooter.setBackSpinSpeed(Shooters.BACKSPIN_SPEED);
    }, () -> {
      shooter.stopShooter();
      shooter.stopBackSpinMotor();
    }, shooter), false);

    // A -> intake ball

    gamepad.getButton(Button.A).whileHeld(setIntakeCommand(Ctake.INTAKE_SPEED), false);

    // B -> Outtake

    gamepad.getButton(Button.B).whileHeld(setIntakeCommand(-Ctake.INTAKE_SPEED), false);

    // X -> Index

    gamepad.getButton(Button.X).whileHeld(setIndexCommand(Constants.INDEX_SPEED), false);

    // Y -> Reverse Index

    gamepad.getButton(Button.Y).whileHeld(setIndexCommand(-Constants.INDEX_SPEED), false);

    // TODO: test face hub
    driveController.getButton(Button.A).whileHeld(faceHub, false);

    // (driveController) left bumper -> toggle intake up / down

    driveController.getButton(Button.LeftBumper).whenPressed(intake::togglePiston,
      intake);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {

    // Voltage constraint makes sure we dont accelerate too fast during auton

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter), DriveConstants.kDriveKinematics,
      10);

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(
        DriveConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);

    return turnCommand;
  }

  private Command setIntakeCommand(double speed) {
    return new RunEndCommand(() -> intake.setIntakeMotor(speed), intake::stop, intake);
  }

  private Command setIndexCommand(double speed) {
    return new RunEndCommand(() -> index.setIndexMotor(speed), index::stop, index);
  }

}
