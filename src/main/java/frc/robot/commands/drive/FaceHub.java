// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.rambots4571.rampage.vision.Limelight;
import com.rambots4571.rampage.vision.ReadValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class FaceHub extends CommandBase {
	private final DriveTrain driveTrain;
	private final PIDController controller;
	private final Limelight limelight = Limelight.getInstance();

	private final double kP = 0.2;
	private final double kI = 0;
	private final double kD = 0;

	private final double maxOutput = 0.7;

	/** Creates a new FaceHub. */
	public FaceHub(DriveTrain driveTrain) {
		this.driveTrain = driveTrain;
		addRequirements(driveTrain);
		controller = new PIDController(kP, kI, kD);
		controller.setTolerance(2.0);
	}

	private boolean hasValidTarget() {
		return limelight.getValue(ReadValue.tv) == 1;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (hasValidTarget())
			controller.setSetpoint(0);
		else
			this.cancel();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		SmartDashboard.putData(controller);

		double xOffset = limelight.getValue(ReadValue.tx);
		if (xOffset == Integer.MAX_VALUE)
			this.cancel();

		double output = MathUtil.clamp(controller.calculate(xOffset), -maxOutput, maxOutput);

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
		return !hasValidTarget() || controller.atSetpoint();
	}
}
