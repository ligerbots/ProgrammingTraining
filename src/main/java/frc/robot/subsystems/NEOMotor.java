// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NEOMotor extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;  

  /** Creates a new NEOMotor. */
  public NEOMotor() {
    // create motor controller Sparkmax, assign CANID to m_motor, type of motor = brushelss
    m_motor = new CANSparkMax(Constants.MOTOR_CAN_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults(); 

    // assigns m_encoder to m_motor
    m_encoder = m_motor.getEncoder();

    // Set the position conversion factor.
    // m_encoder.setPositionConversionFactor(1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_motor.set(0.5);
  }
}
