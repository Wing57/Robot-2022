// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  // Local Talon Variables
  private final WPI_TalonFX rightMaster;
  private final WPI_TalonFX rightMotor2;
  private final WPI_TalonFX rightMotor3;
  
  private final WPI_TalonFX leftMaster;
  private final WPI_TalonFX leftMotor2;
  private final WPI_TalonFX leftMotor3;

  private final MotorControllerGroup leftDrive;
  private final MotorControllerGroup rightDrive;


  // Local drive variable
  private final DifferentialDrive drive;

  

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // Talons
    rightMaster = new WPI_TalonFX(Constants.RIGHT_MOTOR_1);
    rightMotor2 = new WPI_TalonFX(Constants.RIGHT_MOTOR_2);
    rightMotor3 = new WPI_TalonFX(Constants.RIGHT_MOTOR_3);

    leftMaster = new WPI_TalonFX(Constants.LEFT_MOTOR_1);
    leftMotor2 = new WPI_TalonFX(Constants.LEFT_MOTOR_2);
    leftMotor3 = new WPI_TalonFX(Constants.LEFT_MOTOR_3);

    leftDrive = new MotorControllerGroup(leftMaster, leftMotor2, leftMotor3);
    rightDrive = new MotorControllerGroup(rightMaster, rightMotor2, rightMotor3);
  
    
    
    //DifferentialDrive
    drive = new DifferentialDrive(leftDrive, rightDrive);
  }


  @Override
  public void periodic() {

  }
 
  public void driveWithJoysticks(Joystick leftJoystick, Joystick rightJoystick)
  {
    
    drive.tankDrive(leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(1)*-1);
  }
} 