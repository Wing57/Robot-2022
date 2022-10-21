// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Vision;
import frc.robot.subsystems.DriveTrain;

public class FaceHub extends CommandBase {
  private final DriveTrain driveTrain;
  private final PIDController controller;
  private final Vision vision = Vision.getInstance();

  private final double kP = 0.1125;
  private final double kI = 0.196;
  private final double kD = 0.025;

  private final double maxOutput = 0.7;

  /** Creates a new FaceHub. */
  public FaceHub(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    controller = new PIDController(kP, kI, kD);
    controller.setTolerance(2.0);
  }

  @Override
  public void initialize() {
    if (vision.hasValidTarget()) controller.setSetpoint(0);
  }

  @Override
  public void execute() {
    // face hub controller is already added by default on LW
    // SmartDashboard.putData("Face Hub Controller", controller);
    // SmartDashboard.putData("Vision", vision);

    double xOffset = vision.getHorizontalOffset();
    if (xOffset == Integer.MAX_VALUE) this.cancel();

    double output = MathUtil.clamp(controller.calculate(xOffset), -maxOutput, maxOutput);

    driveTrain.drive(-output, output);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return !vision.hasValidTarget() || controller.atSetpoint();
  }
}
