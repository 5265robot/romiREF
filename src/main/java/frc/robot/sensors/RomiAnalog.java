// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RomiAnalog extends SubsystemBase {
  AnalogPotentiometer m_romiInputLeft;
  AnalogPotentiometer m_romiInputRight;

  /** Creates a new RomiAnalog. */
  public RomiAnalog() {
    m_romiInputLeft = new AnalogPotentiometer(Constants.LightInput.leftInput);
    m_romiInputRight = new AnalogPotentiometer(Constants.LightInput.rightInput);
    
  }

  public double getLeftLightSensor() {
      return m_romiInputLeft.get();
  }

  public double getRigthLightSensor(){
    return m_romiInputRight.get();
  }

  public double getDifference(){
    double l = getLeftLightSensor();
    double r = getRigthLightSensor();
    return (l - r); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getLeftLightSensor();
  }
}
