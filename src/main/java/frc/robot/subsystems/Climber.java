// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
	/** Creates a new Climber. */
	private final CANSparkMax Actmotor;
	private final WPI_TalonFX hookMotor;
	private final DigitalInput toplimitswitch;

	public Climber() {
		Actmotor = new CANSparkMax(Constants.ACTUATOR_MOTOR, MotorType.kBrushless);
		hookMotor = new WPI_TalonFX(Constants.HOOK_MOTOR);
		toplimitswitch = new DigitalInput(7);

		Actmotor.restoreFactoryDefaults();
		hookMotor.configFactoryDefault();

		// Corrupted lose your mind

		Actmotor.setOpenLoopRampRate(0.20);
	}
	public void setActmotor(double speed) {
		if  (toplimitswitch.get() && speed > 0.0) speed = 0;
		
		else {
			Actmotor.set(speed);
		}
		
	}

	public void sethookMotor(double speed) {
		hookMotor.set(speed);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
