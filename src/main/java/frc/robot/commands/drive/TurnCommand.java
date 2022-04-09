// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnCommand extends CommandBase {
	private final DriveTrain driveTrain;
	private final PIDController controller;
	private final double angle;

	private final double kP = 0.5;
	private final double kI = 0.0;
	private final double kD = 0.0;

	/** Creates a new TurnCommand. */
	public TurnCommand(DriveTrain driveTrain, double angle) {
		this.driveTrain = driveTrain;
		this.angle = angle;
		addRequirements(driveTrain);
		controller = new PIDController(kP, kI, kD);
		controller.setTolerance(2.0);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		controller.setSetpoint(angle);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		SmartDashboard.putData("Turn Controller", controller);
		double output = MathUtil.clamp(controller.calculate(driveTrain.getAngle()), -0.8,
		  0.8);
		driveTrain.drive(-output, output);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		driveTrain.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return controller.atSetpoint();
	}
}
