// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoTurnTurret extends CommandBase {
	/** Creates a new turret. */
	private Limelight limelight;
	private Shooter shooter;

	public AutoTurnTurret(Limelight limelight, Shooter shooter) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.limelight = limelight;
		this.shooter = shooter;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (limelight.getHorizontalOffset() < -0.5) {
			shooter.setTurretSpeed(-0.2);
		} else if (limelight.getHorizontalOffset() > 0.5) {
			shooter.setTurretSpeed(0.2);
		} else {
			shooter.setTurretSpeed(0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooter.setTurretSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
