// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class FalconPID extends PIDSubsystem {
  private final WPI_TalonFX talon = new WPI_TalonFX(10);

  double selSenPos = talon.getSelectedSensorPosition(0); /* position units */
  double pos_Rotations = (double) selSenPos / Constants.kUnitsPerRevolution;

  /** Creates a new FalconPID. */
  public FalconPID() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.kP, Constants.kI, Constants.kD));
    
    // Reset TalonFX & perform any needed clean up
	  /* newer config API */
		TalonFXConfiguration configs = new TalonFXConfiguration();
		/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
		configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		/* config all the settings */
    talon.configAllSettings(configs);
    talon.setSelectedSensorPosition(0);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if(output > 0.1) {
      talon.set(output);
    } else {
      talon.set(0);
    }
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return pos_Rotations;
  }

  @Override
  public void periodic() {
    // Display distance on Smart Dashboard & print
    SmartDashboard.putNumber("Pos-Rotations: ", pos_Rotations);
  }
}
