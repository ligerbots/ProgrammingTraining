// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NEOMotor extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;  
  private final SparkMaxPIDController m_PIDController;

  // PID Constants for the Neo motor PID controller
  // Set PID proportional value to produce non-zero correction value when robot veers off
  // straight line. P value controls how sensitive the correction is.
  private static final double REACHER_K_P = 0.05;
  private static final double REACHER_K_I = 0.0;
  private static final double REACHER_K_D = 0.0;
  private static final double REACHER_K_FF = 0.0;
  
  /** Creates a new NEOMotor. */
  public NEOMotor() {
    // create motor controller Sparkmax, assign CANID 1 to m_motor, type of motor = brushelss
    m_motor = new CANSparkMax(Constants.MOTOR_CAN_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    // assigns m_encoder to m_motor
    m_encoder = m_motor.getEncoder();

    // assigns PID controller to motor
    m_PIDController = m_motor.getPIDController();
    m_PIDController.setP(REACHER_K_P);
    m_PIDController.setI(REACHER_K_I);
    m_PIDController.setD(REACHER_K_D);
    m_PIDController.setFF(REACHER_K_FF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // should i put anything here?
  }
}
