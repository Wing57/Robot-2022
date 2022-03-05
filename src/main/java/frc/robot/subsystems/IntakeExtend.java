// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeExtend extends SubsystemBase {
	/** Creates a new IntakeExtend. */
	private final DoubleSolenoid piston;
	private final Compressor comp;

	public IntakeExtend() {
		comp = new Compressor(Constants.MODULE_NUMBER, PneumaticsModuleType.REVPH);
		piston = new DoubleSolenoid(Constants.MODULE_NUMBER, PneumaticsModuleType.REVPH,
		  Constants.INTAKE_PISTON_FORWARD_CHANNEL, Constants.INTAKE_PISTON_REVERSE_CHANNEL);

		comp.enableDigital();
	}

	public void togglePiston() {
		Value oppositeValue = piston.get() == Value.kForward ? Value.kReverse
		  : Value.kForward;
		piston.set(oppositeValue);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
