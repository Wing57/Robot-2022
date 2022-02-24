package frc.robot.commands;

import com.rambots4571.rampage.joystick.DriveStick;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class TankDriveCommand extends CommandBase {
	private final DriveTrain driveTrain;

	public TankDriveCommand(DriveTrain driveTrain) {
		this.driveTrain = driveTrain;
		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		driveTrain.drive(RobotContainer.leftStick.getAxisValue(DriveStick.Axis.yAxis),
		  RobotContainer.rightStick.getAxisValue(DriveStick.Axis.yAxis));
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
