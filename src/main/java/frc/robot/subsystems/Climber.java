// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
	/** Creates a new Climber. */
	private final CANSparkMax actMotor;
	private final CANSparkMax actMotor2;
	private final WPI_TalonFX hookMotor;
	private final DigitalInput topLimitSwitch;

	public Climber() {
		actMotor = new CANSparkMax(Constants.ACTUATOR_MOTOR, MotorType.kBrushless);
		actMotor2 = new CANSparkMax(Constants.ACTUATOR_MOTOR2, MotorType.kBrushless);
		hookMotor = new WPI_TalonFX(Constants.HOOK_MOTOR);
		topLimitSwitch = new DigitalInput(7);

		actMotor.restoreFactoryDefaults();
		actMotor2.restoreFactoryDefaults();
		hookMotor.configFactoryDefault();

		hookMotor.setNeutralMode(NeutralMode.Brake);

		actMotor2.follow(actMotor);

		// Corrupted lose your mind

		actMotor.setOpenLoopRampRate(0.20);
	}

	public void setActMotor(double speed) {
		if (topLimitSwitch.get() && speed > 0.0)
			stopActMotor();
		else {
			actMotor.set(speed);
			actMotor2.set(speed);
		}

	}

	public void setHookMotor(double speed) {
		hookMotor.set(speed);
	}

	public void stopActMotor() {
		actMotor.set(0);
		actMotor2.set(0);
	}

	public void stopHookMotor() {
		hookMotor.set(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
