// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class NEOMotor extends TrapezoidProfileSubsystem{
  /** Creates a new NEOMotor. */
  private final CANSparkMax m_motor = new CANSparkMax(2, MotorType.kBrushless);
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_PIDController;

  private static final double K_P = 0.03;
  private static final double K_I = 0.0;
  private static final double K_D = 0.0;
  private static final double K_FF = 0.0;

  private static final double V_MAX = Math.toRadians(720.0*3); // rad/s
  private static final double ACC_MAX = Math.toRadians(360.0*5); // rad/s^2

  public NEOMotor() {
    super(new Constraints(V_MAX, ACC_MAX));

    m_motor.restoreFactoryDefaults();
    // test

    m_encoder = m_motor.getEncoder();
    
    m_encoder.setPosition(0.0);

    m_encoder.setPositionConversionFactor(2*Math.PI);
    
    m_PIDController = m_motor.getPIDController();
    m_PIDController.setP(K_P);
    m_PIDController.setI(K_I);
    m_PIDController.setD(K_D);
    m_PIDController.setFF(K_FF);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("encoderReading", Math.toDegrees(getAngle()));
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    m_motor.set(speed);
  }

  // return angle in radians
  public double getAngle(){
    return m_encoder.getPosition();
  }

  public void setAngle(double angle){
    m_PIDController.setReference(angle, ControlType.kPosition, 0, K_FF); 
  }

  public void setAngleGoal(double angle){
    super.setGoal(new State(angle, 0.0));
  }

  @Override
  protected void useState(State state) {
    // TODO Auto-generated method stub
    setAngle(state.position);
    SmartDashboard.putNumber("angle setpoint", state.position);
  }
}
