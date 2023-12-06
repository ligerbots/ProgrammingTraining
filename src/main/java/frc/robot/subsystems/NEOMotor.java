// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class NEOMotor extends TrapezoidProfileSubsystem {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;  
  private final SparkMaxPIDController m_PIDController;

  private static final double K_P = 0.05;
  private static final double K_I = 0.0;
  private static final double K_D = 0.0;
  private static final double K_FF = 0.0;

  // *Note: Neo reads a raw percentage of a full revolution [0, 1]
  // So this conversion factor is just how much measurement unit is one full revolution
  private final double RADIAN_PER_REVOLUTION = 2*Math.PI;

  static final double MAX_VEL_RADIAN_PER_SEC = Math.toRadians(120.0);
  static final double MAX_ACC_RADIAN_PER_SEC_SQ = Math.toRadians(30.0);


  /** Creates a new NEOMotor. */
  public NEOMotor() {
    super(new TrapezoidProfile.Constraints(MAX_VEL_RADIAN_PER_SEC, MAX_ACC_RADIAN_PER_SEC_SQ));

    // create motor controller Sparkmax, assign CANID to m_motor, type of motor = brushelss
    m_motor = new CANSparkMax(Constants.MOTOR_CAN_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    // assigns m_encoder to m_motor
    m_encoder = m_motor.getEncoder();

    m_encoder.setPosition(0.0);

    // Set the position conversion factor.
    m_encoder.setPositionConversionFactor(RADIAN_PER_REVOLUTION);

    m_PIDController = m_motor.getPIDController();
    m_PIDController.setP(K_P);
    m_PIDController.setI(K_I);
    m_PIDController.setD(K_D);
    m_PIDController.setFF(K_FF);
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

  public void setAngle(double angle){
    m_PIDController.setReference(angle, ControlType.kPosition, 0, K_FF); 
  }

  @Override
  protected void useState(State state) {
    // TODO Auto-generated method stub
    setAngle(state.position);
  }

  // set the angle to angles in radians
  public void setAngleGoal(double angle){
    super.setGoal(new State(angle, 0.0));
  }
}
