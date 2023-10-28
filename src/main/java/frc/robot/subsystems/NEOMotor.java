// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NEOMotor extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;  

  // *Note: Neo reads a raw percentage of a full revolution [0, 1]
  // So this conversion factor is just how much measurement unit is one full revolution
  private final double RADIAN_PER_REVOLUTION = 2*Math.PI;

  /** Creates a new NEOMotor. */
  public NEOMotor() {
    // create motor controller Sparkmax, assign CANID to m_motor, type of motor = brushelss
    m_motor = new CANSparkMax(Constants.MOTOR_CAN_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults(); 

    // assigns m_encoder to m_motor
    m_encoder = m_motor.getEncoder();

    m_encoder.setPosition(0.0);

    // Set the position conversion factor.
    m_encoder.setPositionConversionFactor(RADIAN_PER_REVOLUTION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoderReading", Math.toDegrees(getAngle()));
  }

  public double getAngle(){
    return m_encoder.getPosition();
  }

  public void setSpeed(double speed){
    m_motor.set(speed);
  }
}
