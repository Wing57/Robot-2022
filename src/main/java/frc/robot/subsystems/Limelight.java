package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem encapsulates the Limelight camera and
 * its NetworkTables interface.
 * The Limelight is integral to our automated shooting system.
 * It provides both the horizontal offset for turret alignment
 * and distance to target for shooter revving.
 */
public class Limelight extends SubsystemBase {

	NetworkTable m_limelightTable;
	NetworkTableEntry ty, tx, ta;

	public void Vision() {
		m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

	}

	public enum CamMode {

	}

	@Override
	public void periodic() {

		ty = m_limelightTable.getEntry("ty");
		tx = m_limelightTable.getEntry("tx");
		ta = m_limelightTable.getEntry("ta");

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);
		double targetOffsetAngle_Vertical = ty.getDouble(0.0);

		// how many degrees back is your limelight rotated from perfectly vertical?
		double limelightMountAngleDegrees = 25.0;

		// distance from the center of the Limelight lens to the floor
		double limelightLensHeightInches = 20.0;

		// distance from the target to the floor
		double goalHeightInches = 60.0;

		double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
		double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

		// calculate distance
		double distanceFromLimelightToGoalInches = (goalHeightInches
		  - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);
		SmartDashboard.putNumber("DistanceToTarget", distanceFromLimelightToGoalInches);

	}

	public void execute() {

	}
}
