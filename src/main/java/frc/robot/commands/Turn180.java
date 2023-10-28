// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NEOMotor;

public class Turn180 extends CommandBase {
  /** Creates a new Turn180. */
  private NEOMotor m_neo;
  public Turn180(NEOMotor neo) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_neo = neo;
    addRequirements(m_neo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_neo.setSpeed(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_neo.setSpeed(0.01);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_neo.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_neo.getAngle() - Math.PI) <= Math.toRadians(5.0);
  }
}
