// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
	// Local Talon Variables
	private final WPI_TalonFX rightMaster;
	private final WPI_TalonFX leftMaster;

	private final WPI_TalonFX rightMotor2;
	private final WPI_TalonFX rightMotor3;
	private final WPI_TalonFX leftMotor2;
	private final WPI_TalonFX leftMotor3;

	private final DoubleSolenoid shifter;

	private final DifferentialDrive drive;

	/**
	 * Creates a new DriveTrain.
	 */
	public DriveTrain() {
		// Talons
		rightMaster = new WPI_TalonFX(Constants.RIGHT_MOTOR_1);
		rightMotor2 = new WPI_TalonFX(Constants.RIGHT_MOTOR_2);
		rightMotor3 = new WPI_TalonFX(Constants.RIGHT_MOTOR_3);

		leftMaster = new WPI_TalonFX(Constants.LEFT_MOTOR_1);
		leftMotor2 = new WPI_TalonFX(Constants.LEFT_MOTOR_2);
		leftMotor3 = new WPI_TalonFX(Constants.LEFT_MOTOR_3);

		rightMotor2.follow(rightMaster);
		rightMotor2.setInverted(InvertType.FollowMaster);
		rightMotor3.follow(rightMaster);
		rightMotor3.setInverted(InvertType.FollowMaster);

		leftMotor2.follow(leftMaster);
		leftMotor2.setInverted(InvertType.FollowMaster);
		leftMotor3.follow(leftMaster);
		leftMotor3.setInverted(InvertType.FollowMaster);

		// Same as set invert = false
		TalonFXInvertType m_left_invert = TalonFXInvertType.Clockwise;

		// Same as set invert = true
		TalonFXInvertType m_right_invert = TalonFXInvertType.CounterClockwise;

		leftMaster.setInverted(m_left_invert);
		rightMaster.setInverted(m_right_invert);

		// DifferentialDrive
		drive = new DifferentialDrive(leftMaster, rightMaster);

		shifter = new DoubleSolenoid(Constants.MODULE_NUMBER, PneumaticsModuleType.REVPH,
		  Constants.SHIFTER_FORWARD_CHANNEL, Constants.SHIFTER_REVERSE_CHANNEL);
	}

	@Override
	public void periodic() {
	}

	public void driveWithJoysticks(XboxController controller) {
		drive.tankDrive(controller.getRawAxis(1) * -1, controller.getRawAxis(5));
	}

	public void drive(double left, double right) {
		drive.tankDrive(left, right);
	}

	public void stopMotors() {
		drive.stopMotor();
	}

	public void shiftGears() {
		Value oppValue = shifter.get() == Value.kForward ? Value.kReverse : Value.kForward;
		shifter.set(oppValue);
	}
}
