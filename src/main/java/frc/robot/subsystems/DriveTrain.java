package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveTrain extends SubsystemBase {


    private CANSparkMax m_leftLeader = new CANSparkMax(Constants.LEADER_LEFT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(Constants.FOLLOWER_LEFT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax m_rightLeader = new CANSparkMax(Constants.LEADER_RIGHT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(Constants.FOLLOWER_RIGHT_CAN_ID, MotorType.kBrushless);


    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftLeader, m_leftFollower);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightLeader, m_rightFollower);


    private DifferentialDrive m_differentialDrive;


    private AHRS m_navX;

    private final SparkMaxPIDController m_leftPIDController;
    private final SparkMaxPIDController m_rightPIDController;


    public DriveTrain() {
        // setup PID control for TalonFX
        m_leftLeader.restoreFactoryDefaults();

        m_leftPIDController = m_rightLeader.getPIDController();
        m_leftPIDController.setP(Constants.DRIVETRAIN_KP);
        m_leftPIDController.setI(Constants.DRIVETRAIN_KI);
        m_leftPIDController.setD(Constants.DRIVETRAIN_KD);
        m_leftPIDController.setFF(Constants.DRIVETRAIN_KF);

        m_rightLeader.restoreFactoryDefaults();

        m_rightPIDController = m_rightLeader.getPIDController();
        m_rightPIDController.setP(Constants.DRIVETRAIN_KP);
        m_rightPIDController.setI(Constants.DRIVETRAIN_KI);
        m_rightPIDController.setD(Constants.DRIVETRAIN_KD);
        m_rightPIDController.setFF(Constants.DRIVETRAIN_KF);

        m_rightMotors.setInverted(true); // to make both left and right rotate same direction
   
       
        m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_differentialDrive.setSafetyEnabled(false);


        m_navX = new AHRS(Port.kMXP, (byte) 200);
    }

    // Get Gyro info
    public double getGyroAngle() {
        // returns remainder of teh angle / 360
        return Math.IEEEremainder(m_navX.getAngle(), 360) * -1;
    }


    @Override
    public void periodic() {
    }

    // for drive command later
    public void drive(double throttle, double rotate, boolean squaredInput) { 
        m_differentialDrive.arcadeDrive(throttle, -rotate, squaredInput);
    }

    // can past voltage in directly
    public void tankDriveVolts (double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(rightVolts);
        m_differentialDrive.feed();
    }
}




