// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.rambots4571.rampage.joystick.Gamepad;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
	private final WPI_TalonFX shooterMotor, backSmotor;
	private final WPI_TalonSRX turretMotor;
	private final TalonFXInvertType BackspinInvert;
	private final DigitalInput rightturretlimitswitch;
	private final DigitalInput leftturretlimitswitch;

	public Shooter() {
		shooterMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_1);
		turretMotor = new WPI_TalonSRX(Constants.TURRET_MOTOR);
		backSmotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_2);
		rightturretlimitswitch = new DigitalInput(Constants.RIGHTTURRET_LIMITSWITCH_PORT);
		leftturretlimitswitch = new DigitalInput(Constants.LEFTTURRET_LIMITSWITCH_PORT);

		shooterMotor.configFactoryDefault();
		backSmotor.configFactoryDefault();
		turretMotor.configFactoryDefault();

		shooterMotor.setNeutralMode(NeutralMode.Coast);
		turretMotor.setNeutralMode(NeutralMode.Brake);

		BackspinInvert = TalonFXInvertType.CounterClockwise;

		backSmotor.setInverted(BackspinInvert);

		shooterMotor.configOpenloopRamp(0.5);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void setShooterSpeed(double speed) {
		shooterMotor.set(speed);

	}

	public void setBackSpinSpeed(double speed) {
		backSmotor.set(speed);

	}

	public void setTurretSpeed(double speed) {
		if (!rightturretlimitswitch.get()
		  && RobotContainer.gamepad.getAxisValue(Gamepad.Axis.LeftYAxis) < 0)
			speed = 0.0;
		else if (!leftturretlimitswitch.get()
		  && RobotContainer.gamepad.getAxisValue(Gamepad.Axis.LeftYAxis) > 0)
			speed = 0.0;

		else {
			turretMotor.set(speed);
		}
	}

	public void stopShooter() {
		shooterMotor.set(0);

	}

	public void stopBackSpinMotor() {
		backSmotor.set(0);

	}
}
