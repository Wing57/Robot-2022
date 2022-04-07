package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class TestCommandGroup extends SequentialCommandGroup {
	public TestCommandGroup(DriveTrain dt, Shooter shooter) {
		addCommands(new DriveTimedCommand(dt, 5), new AutoShoot(shooter));
	}
}
