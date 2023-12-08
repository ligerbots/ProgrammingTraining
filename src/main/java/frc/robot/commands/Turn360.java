// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NEOMotor;

public class Turn360 extends CommandBase {

  private final NEOMotor m_neo;
  private final double ANGLE_TO_TURN = Math.toRadians(360*10); //2pi

  /** Creates a new Turn360. */
  public Turn360(NEOMotor neo) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_neo = neo;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_neo.setSpeed(0.02);
    m_neo.setAngleGoal(ANGLE_TO_TURN); // setAngle(2pi)
    System.out.println("Turn360 Command executed!!!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_neo.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.toDegrees(m_neo.getAngle()) > 360.0;
  }
}
