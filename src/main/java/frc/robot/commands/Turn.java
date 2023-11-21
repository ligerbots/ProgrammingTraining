// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NEOMotor;

public class Turn extends CommandBase {
  /** Creates a new Turn. */
  private NEOMotor m_neo;
  private DoubleSupplier m_doubleSupplier;

  public Turn(NEOMotor neo, DoubleSupplier doubleSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_neo = neo;
    m_doubleSupplier = doubleSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_neo.setSpeed(m_doubleSupplier.getAsDouble());
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
