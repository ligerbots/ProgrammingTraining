// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Motor extends SubsystemBase {

  CANSparkMax m_motor = new CANSparkMax(Constants.MOTOR_ID, MotorType.kBrushless);

  Encoder m_encoder = new Encoder(Constants.ENCODER_PORT[0], Constants.ENCODER_PORT[1]);

  PIDController m_motorController = new PIDController(1, 0, 0);

  SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(0, 0);

  /** Creates a new Motor. */
  public Motor() {
    m_encoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public double getDistance(){
    return m_encoder.getDistance();
  }

  public void drive(double volt){
    volt = m_feedForward.calculate(volt) + m_motorController.calculate(m_encoder.getRate(), volt);
    m_motor.setVoltage(volt);
  }
}
