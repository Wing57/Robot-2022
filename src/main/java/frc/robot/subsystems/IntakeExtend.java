// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeExtend extends SubsystemBase {
	/** Creates a new IntakeExtend. */
	private final DoubleSolenoid piston;

	public IntakeExtend() {
		Compressor comp = new Compressor(2, PneumaticsModuleType.REVPH);
		piston = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 1, 0);
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
