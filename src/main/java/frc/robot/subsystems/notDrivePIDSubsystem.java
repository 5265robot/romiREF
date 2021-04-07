// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class notDrivePIDSubsystem extends PIDSubsystem {
  private final AnalogPotentiometer m_sensor;
  private final Drivetrain m_drivetrain;

  /** Creates a new DrivePIDSubsystem. */
  public notDrivePIDSubsystem(Drivetrain drivetrain, AnalogPotentiometer sensor) {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    m_sensor = sensor;
    m_drivetrain = drivetrain;
    // addRequirements(m_drivetrain,m_sensor);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_sensor.get();
  }
}
