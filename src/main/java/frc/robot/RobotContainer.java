// Copyright (c) Darius and other Darius contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file and DARI lisence in the root directory of this
// project.

package frc.robot;

import com.rambots4571.rampage.command.RunEndCommand;
import com.rambots4571.rampage.joystick.Controller;
import com.rambots4571.rampage.joystick.Gamepad;
import com.rambots4571.rampage.joystick.Gamepad.Button;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.Ctake;
import frc.robot.Constants.Shooters;
import frc.robot.commands.auton.FourRareFish;
import frc.robot.commands.auton.TurnCommand;
import frc.robot.commands.drive.FaceHub;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.shooter.SetShooterRPM;
import frc.robot.commands.shooter.ShootBall;
import frc.robot.commands.shooter.TarmacShot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  // joysticks
  public static final Controller<Gamepad.Button, Gamepad.Axis> gamepad =
      Gamepad.make(Constants.XBOXCONTROLLER);
  public static final Controller<Gamepad.Button, Gamepad.Axis> driveController =
      Gamepad.make(Constants.XBOXCONTROLLER2);

  // subsystems
  public final DriveTrain driveTrain;
  public final Shooter shooter;
  public final Intake intake;
  public final Index index;

  // commands
  private final TankDriveCommand tankDriveCommand;
  public static FaceHub faceHub;

  public static ShootBall shootBall;
  public static TarmacShot tarmacShot;
  public static SetShooterRPM setShooterRPM;

  public static TurnCommand turnCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain = new DriveTrain();
    shooter = new Shooter();
    intake = new Intake();
    index = new Index();

    // (driveController) X -> SHIFT GEARS
    tankDriveCommand =
        new TankDriveCommand(driveTrain, driveController.getButton(Gamepad.Button.X));

    driveTrain.setDefaultCommand(tankDriveCommand);

    faceHub = new FaceHub(driveTrain);

    shootBall = new ShootBall(shooter);

    tarmacShot = new TarmacShot(shooter);


    setShooterRPM = new SetShooterRPM(shooter, 2944, 2400);

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // right bumper -> shoot ball

    gamepad
        .getButton(Gamepad.Button.RightBumper)
        .whileHeld(
            new RunEndCommand(
                () -> {
                  shooter.setShooterSpeed(Shooters.SHOOT_SPEED);
                  shooter.setBackSpinSpeed(Shooters.BACKSPIN_SPEED);
                },
                () -> {
                  shooter.stopShooter();
                  shooter.stopBackSpinMotor();
                },
                shooter),
            false);
    driveController
        .getButton(Gamepad.Button.RightBumper)
        .whileHeld(
            new RunEndCommand(
                () -> {
                  shooter.setShooterVelocity(Shooters.SHOOT_SPEED);
                },
                () -> {
                  shooter.stopShooter();
                },
                shooter),
            false);

    // A -> intake ball

    gamepad.getButton(Button.A).whileHeld(setIntakeCommand(Ctake.INTAKE_SPEED), false);

    // B -> Outtake

    gamepad.getButton(Button.B).whileHeld(setIntakeCommand(-Ctake.INTAKE_SPEED), false);

    // X -> Index

    gamepad.getButton(Button.X).whileHeld(setIndexCommand(Constants.INDEX_SPEED), false);

    // Y -> Reverse Index

    gamepad.getButton(Button.Y).whileHeld(setIndexCommand(-Constants.INDEX_SPEED), false);

    // Left Bumper -> Shoot Anywhere

    gamepad.getButton(Gamepad.Button.LeftBumper).whileHeld(setShooterRPM, false);

    // (driverController) A -> Face Hub

    driveController.getButton(Button.A).whileHeld(faceHub, false);

    // (driveController) left bumper -> toggle intake up / down

    driveController.getButton(Button.LeftBumper).whenPressed(intake::togglePiston, intake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    autonChooser.addOption("4ball", new FourRareFish(this));

    SmartDashboard.putData("autonChooser", autonChooser);

    return autonChooser.getSelected();
  }

  private Command setIntakeCommand(double speed) {
    return new RunEndCommand(() -> intake.setIntakeMotor(speed), intake::stop, intake);
  }

  private Command setIndexCommand(double speed) {
    return new RunEndCommand(() -> index.setIndexMotor(speed), index::stop, index);
  }
}
