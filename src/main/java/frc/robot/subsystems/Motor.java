// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Motor extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.MOTOR_CAN_ID, MotorType.kBrushless);
  private final Encoder m_encoder = new Encoder(Constants.ENCODER_PORT[0], Constants.ENCODER_PORT[1]);

  private EncoderSim m_encoderSim;

  private final FlywheelSim m_motorSim = new FlywheelSim(
      LinearSystemId.identifyVelocitySystem(Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
      DCMotor.getNEO(1),
      Constants.kDriveGearRatio);

  private final PIDController m_motorController = 
      new PIDController(Constants.kP, Constants.kI, Constants.kD);

    private final ElevatorFeedforward m_feedforward = 
    new ElevatorFeedforward(Constants.kS, Constants.kG, Constants.kV, Constants.kA);
  

  /** Creates a new Motor. */
  public Motor() {
    m_encoderSim = new EncoderSim(m_encoder);

    m_encoder.setDistancePerPulse(Constants.kDriveEncoderDistancePerPulse);
  }

  @Override
  public void periodic() {
    double output = SmartDashboard.getNumber("motor speed", 0.0);

    // TODO: try to put these two output onto smartdashboard and see why the distance decreases to negative infinity even when speed is 0
    output = m_motorController.calculate(m_encoderSim.getRate(), output) + m_feedforward.calculate(output);
    // This method will be called once per scheduler run
    m_motorSim.setInputVoltage(output);

    m_motorSim.update(0.02);
    m_encoderSim.setRate(m_motorSim.getAngularVelocityRPM());
    m_encoderSim.setDistance(getDistance() + m_encoderSim.getRate()*0.02);

    SmartDashboard.putNumber("Distance Traveled (in meters)", getDistance());
  }

  // returned in meters
  public double getDistance() {
    return m_encoderSim.getDistance();
  }

    public void resetEncoders() {
      m_encoder.reset();
    }
}
