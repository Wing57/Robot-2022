// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  // Local Talon Variables
  WPI_TalonFX righMotor1;
  WPI_TalonFX rightMotor2;
  WPI_TalonFX rightMotor3;
  
  WPI_TalonFX leftMotor1;
  WPI_TalonFX leftMotor2;
  WPI_TalonFX leftMotor3;

  // Local MotorControllers
  MotorControllerGroup rightMotors;
  MotorControllerGroup leftMotors;

  // Local drive variable
  DifferentialDrive drive;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // Talons
    WPI_TalonFX rightMotor1 = new WPI_TalonFX(Constants.RIGHT_MOTOR_1);
    WPI_TalonFX rightMotor2 = new WPI_TalonFX(Constants.RIGHT_MOTOR_2);
    WPI_TalonFX rightMotor3 = new WPI_TalonFX(Constants.RIGHT_MOTOR_3);

    WPI_TalonFX leftMotor1 = new WPI_TalonFX(Constants.LEFT_MOTOR_1);
    WPI_TalonFX leftMotor2 = new WPI_TalonFX(Constants.LEFT_MOTOR_2);
    WPI_TalonFX leftMotor3 = new WPI_TalonFX(Constants.lEFT_MOTOR_3);

    //Talon Motor groups
    MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);
    MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);
    
    //DifferentialDrive
    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void periodic() {

  }
 

  public void driveWithJoysticks(XboxController controller, double speed)
  {
    
    drive.arcadeDrive(controller.getRawAxis(Constants.XBOX_LEFT_Y_AXIS)*speed, controller.getRawAxis(Constants.XBOX_LEFT_X_AXIS)*speed);
  }

  public void driveForward(double speed)
  {
    drive.tankDrive(speed, speed);
  }

  public void stop()
  {
    drive.stopMotor();
  }

} 
