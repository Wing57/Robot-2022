// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.rambots4571.rampage.vision.Limelight;
import com.rambots4571.rampage.vision.Limelight.CamMode;
import com.rambots4571.rampage.vision.Limelight.LedMode;
import com.rambots4571.rampage.vision.ReadValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooters;

public class Vision extends SubsystemBase {
	private final Limelight limelight = Limelight.getInstance();

	/** Creates a new Limelight. */
	public Vision() {
	}

	// Gets the number of valid targets (0 or 1)
	public boolean hasValidTarget() {
		return limelight.getValue(ReadValue.tv) == 1;
	}

	// Gets the horizontal offset of the center crosshair from the target
	public double getHorizontalOffset() {
		return limelight.getValue(ReadValue.tx);
	}

	// Gets the vertical offset of the center crosshair from the target
	public double getVerticalOffset() {
		return limelight.getValue(ReadValue.ty);
	}

	// Gets the estimated distance from the robot to the target
	public double getEstimatedDistance() {
		double offset = getVerticalOffset();
		// return 0.0;
		return Shooters.LIMELIGHT_DISTANCE_K[0] * Math.pow(offset, 2)
		  + Shooters.LIMELIGHT_DISTANCE_K[1] * offset + Shooters.LIMELIGHT_DISTANCE_K[2];
	}

	// Gets the active pipeline (0 to 9)
	public int getPipe() {
		return limelight.getEntry("rambotspipeline").getNumber(-1).intValue();
	}

	// Sets the active pipeline (0 to 9)
	public void setPipe(int pipeNum) {
		limelight.setPipeline(pipeNum);
		;
	}

	// Sets the LED state. LEDs on = 3, LEDs off = 1
	public void setLEDMode(LedMode mode) {
		limelight.setLedMode(mode);
	}

	// Set's the limelight's state of operation. Visiom Processing = 0, Driver
	// Camera = 1
	public void setCameraMode(CamMode mode) {
		limelight.setCamMode(mode);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
	}
}
