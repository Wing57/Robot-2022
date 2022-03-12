// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
	/** Creates a new Climber. */
	private final WPI_TalonFX Actmotor;
	private final DoubleSolenoid ClimbPiston;

	public Climber() {
		Actmotor = new WPI_TalonFX(Constants.ACTUATOR_MOTOR);
		ClimbPiston = new DoubleSolenoid(Constants.MODULE_NUMBER, PneumaticsModuleType.REVPH,
		  Constants.CLIMBER_PISTON_FORWARD_CHANNEL, Constants.CLIMBER_PISTON_REVERSE_CHANNEL);
	}

	public void setActmotor(double speed) {
		Actmotor.set(speed);
	}

	public void toggleClimber() {
		Value oppValue = ClimbPiston.get() == Value.kForward ? Value.kReverse
		  : Value.kForward;
		ClimbPiston.set(oppValue);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
