// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_Right;
  private final Supplier<Double> m_Left; 
  


  public TankDrive(Drivetrain drivetrain, 
  Supplier<Double> Right,
  Supplier<Double> Left) { 
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_Right = Right;
    m_Left = Left;
    
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivetrain.tankDrive(m_Left.get(), m_Right.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
