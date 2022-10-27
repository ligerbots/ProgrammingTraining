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
	private final CANSparkMax m_motor = new CANSparkMax(Constants.MOTOR_CAN_ID, MotorType.kBrushless);
	private final Encoder m_encoder = new Encoder(Constants.ENCODER_PORT[0], Constants.ENCODER_PORT[1], false);

	private final PIDController m_motorController = new PIDController(Constants.kP, Constants.kI, Constants.kD);

	private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV,
			Constants.kA);

	/** Creates a new Motor. */
	public Motor() {
		m_encoder.setDistancePerPulse(Constants.kDriveEncoderDistancePerPulse);
	}

	@Override
	public void periodic() {
	}

	public void drive(double velocity) {
		m_motor.setVoltage(
				m_feedforward.calculate(velocity) + m_motorController.calculate(m_encoder.getRate(), velocity));
	}

	public double getDistance() {
		return m_encoder.getDistance();
	}

	public void resetEncoders() {
		m_encoder.reset();
	}
}
