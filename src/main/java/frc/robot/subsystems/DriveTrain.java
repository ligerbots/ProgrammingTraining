package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {

    private WPI_TalonFX m_leftLeader = new WPI_TalonFX(Constants.LEADER_LEFT_CAN_ID);
    private WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.FOLLOWER_LEFT_CAN_ID);
    private WPI_TalonFX m_rightLeader = new WPI_TalonFX(Constants.LEADER_RIGHT_CAN_ID);
    private WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.FOLLOWER_RIGHT_CAN_ID);

    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftLeader, m_leftFollower);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightLeader, m_rightFollower);

    private DifferentialDrive m_differentialDrive;

    private AHRS m_navX;

    public DriveTrain() {
        // setup PID control for TalonFX
        m_leftLeader.configFactoryDefault();
        m_leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        m_leftLeader.set(ControlMode.Position,0);
        m_leftLeader.config_kP(0, Constants.DRIVETRAIN_KP);
        m_leftLeader.config_kI(0, Constants.DRIVETRAIN_KI);
        m_leftLeader.config_kD(0, Constants.DRIVETRAIN_KD);
        m_leftLeader.config_kF(0, Constants.DRIVETRAIN_KF);
        m_leftLeader.setSensorPhase(false);

        m_rightLeader.configFactoryDefault();
        m_rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        m_rightLeader.set(ControlMode.Position,0);
        m_rightLeader.config_kP(0, Constants.DRIVETRAIN_KP);
        m_rightLeader.config_kI(0, Constants.DRIVETRAIN_KI);
        m_rightLeader.config_kD(0, Constants.DRIVETRAIN_KD);
        m_rightLeader.config_kF(0, Constants.DRIVETRAIN_KF);
        m_rightLeader.setSensorPhase(true);
        
        m_rightMotors.setInverted(true);
        setMotorMode(NeutralMode.Coast);
       
        m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_differentialDrive.setSafetyEnabled(false);
        
        m_navX = new AHRS(Port.kMXP, (byte) 200);
    }

    public void setMotorMode(NeutralMode m) {
        m_leftLeader.setNeutralMode(m);
        m_leftFollower.setNeutralMode(m);
        m_rightLeader.setNeutralMode(m);
        m_rightFollower.setNeutralMode(m); 
    }

    // Get the current set speed of the speed controllers
    public double getRightSpeed() {
        return -m_rightMotors.get();
    }

    public double getLeftSpeed() {
        return m_leftMotors.get();
    }

    // Get stats about the encoders
    public double getLeftEncoderDistance() {
        return m_leftLeader.getSelectedSensorPosition() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT;
    }
    public void setLeftEncoderDistance(double distance) {
        m_leftLeader.setSelectedSensorPosition((int) (distance / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT));
    }

    public double getRightEncoderDistance() {
        // return m_rightEncoder.getDistance();
        return -m_rightLeader.getSelectedSensorPosition() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT;
    }
    public void setRightEncoderDistance(double distance) {
        m_rightLeader.setSelectedSensorPosition((int) (-distance / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT));
    }

    public double getDistance() {
        return 0.5 * (getLeftEncoderDistance() + getRightEncoderDistance());
    }

    public double getLeftEncoderVelocity() {
        // sensor velocity is per 100ms, so an extra scale of 10
        return m_leftLeader.getSelectedSensorVelocity() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT * 10.0;
    }
    public double getRightEncoderVelocity() {
        // sensor velocity is per 100ms, so an extra scale of 10
        return -m_rightLeader.getSelectedSensorVelocity() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT * 10.0;
    }

    public int getLeftEncoderTicks() {
        // return m_leftEncoder.get();
        return (int)m_leftLeader.getSelectedSensorPosition();
    }
    public void setLeftEncoderTicks(int ticks){
        m_leftLeader.setSelectedSensorPosition(ticks);
    }
    public int getRightEncoderTicks() {
        // return m_rightEncoder.get();
        return (int)m_rightLeader.getSelectedSensorPosition();
    }
    public void setRightEncoderTicks(int ticks){
        m_rightLeader.setSelectedSensorPosition(ticks);
    }

    // Get Gyro info
    public double getGyroAngle() {
        return Math.IEEEremainder(m_navX.getAngle(), 360) * -1;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("driveTrain/NavX gyro", getGyroAngle());

        SmartDashboard.putNumber("driveTrain/left encoder", getLeftEncoderTicks());
        SmartDashboard.putNumber("driveTrain/right encoder", getRightEncoderTicks());
        SmartDashboard.putNumber("driveTrain/left distance", getLeftEncoderDistance());
        SmartDashboard.putNumber("driveTrain/right distance", getRightEncoderDistance());

        // SmartDashboard.putNumber("driveTrain/LeftFollower", m_leftFollower.getSelectedSensorPosition());
        // SmartDashboard.putNumber("driveTrain/LeftLeader", m_leftLeader.getSelectedSensorPosition());
        // SmartDashboard.putNumber("driveTrain/RightFollower", m_rightFollower.getSelectedSensorPosition());
        // SmartDashboard.putNumber("driveTrain/RightLeader", m_rightLeader.getSelectedSensorPosition());
    }

    public void drive(double throttle, double rotate, boolean squaredInput) {
        // can anyone think of why the rotate is negated? XboxController!
        m_differentialDrive.arcadeDrive(throttle, -rotate, squaredInput);
    }
}
