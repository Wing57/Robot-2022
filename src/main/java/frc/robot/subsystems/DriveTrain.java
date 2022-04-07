// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

	private final SlewRateLimiter filter = new SlewRateLimiter(0.5);

	private final SupplyCurrentLimitConfiguration currentLimitConfig =
	  new SupplyCurrentLimitConfiguration(true, 40, 60, 4);

	private final NeutralMode neutralMode = NeutralMode.Brake;
	private final double rampRate = 1.0;

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

		// Factory Resets all TalonFX

		rightMaster.configFactoryDefault();
		rightMotor2.configFactoryDefault();
		rightMotor3.configFactoryDefault();

		leftMaster.configFactoryDefault();
		leftMotor2.configFactoryDefault();
		leftMotor3.configFactoryDefault();

		// Sets Motor to Brake/Coast

		rightMaster.setNeutralMode(neutralMode);
		rightMotor2.setNeutralMode(neutralMode);
		rightMotor3.setNeutralMode(neutralMode);

		leftMaster.setNeutralMode(neutralMode);
		leftMotor2.setNeutralMode(neutralMode);
		leftMotor3.setNeutralMode(neutralMode);

		// Current limit to prevent breaker tripping. Approx at 150% of rated
		// current supply.

		rightMaster.configSupplyCurrentLimit(currentLimitConfig);
		rightMotor2.configSupplyCurrentLimit(currentLimitConfig);
		rightMotor3.configSupplyCurrentLimit(currentLimitConfig);

		leftMaster.configSupplyCurrentLimit(currentLimitConfig);
		leftMotor2.configSupplyCurrentLimit(currentLimitConfig);
		leftMotor3.configSupplyCurrentLimit(currentLimitConfig);

		// Same as set invert = false
		TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise;

		// Same as set invert = true
		TalonFXInvertType rightInvert = TalonFXInvertType.CounterClockwise;

		leftMaster.setInverted(leftInvert);
		rightMaster.setInverted(rightInvert);

		rightMotor2.follow(rightMaster);
		rightMotor2.setInverted(InvertType.FollowMaster);
		rightMotor3.follow(rightMaster);
		rightMotor3.setInverted(InvertType.FollowMaster);

		leftMotor2.follow(leftMaster);
		leftMotor2.setInverted(InvertType.FollowMaster);
		leftMotor3.follow(leftMaster);
		leftMotor3.setInverted(InvertType.FollowMaster);

		// Ramping motor output to prevent instantaneous directional changes (Values
		// need testing)
		rightMaster.configOpenloopRamp(rampRate, 25);
		rightMotor2.configOpenloopRamp(rampRate, 25);
		rightMotor3.configOpenloopRamp(rampRate, 25);

		leftMaster.configOpenloopRamp(rampRate, 25);
		leftMotor2.configOpenloopRamp(rampRate, 25);
		leftMotor3.configOpenloopRamp(rampRate, 25);

		// DifferentialDrive
		drive = new DifferentialDrive(leftMaster, rightMaster);

		shifter = new DoubleSolenoid(Constants.MODULE_NUMBER, PneumaticsModuleType.REVPH,
		  Constants.SHIFTER_FORWARD_CHANNEL, Constants.SHIFTER_REVERSE_CHANNEL);
	}

	@Override
	public void periodic() {
	}

	public void drive(double left, double right) {
		drive.tankDrive(filter.calculate(left), filter.calculate(right));
	}

	public void stopMotors() {
		drive.stopMotor();
	}

	public void shiftGears() {
		Value oppValue = shifter.get() == Value.kForward ? Value.kReverse : Value.kForward;
		shifter.set(oppValue);
	}
}
