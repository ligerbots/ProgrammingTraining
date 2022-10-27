// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int MOTOR_CAN_ID = 8;
    public static final int[] ENCODER_PORT = {0, 1};

    public static final double kWheelDiameterMeters = 0.15;
    public static final int kEncoderCPR = 2048;
    public static final double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double ksVolts = 0.587;
    public static final double kvVoltSecondsPerMeter = 2.3;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0917;

    public static final double kDriveGearRatio = 8.16;

    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Feedforward constants
    public static final double kS = 0.182;
    // The following constants are computed from https://www.reca.lc/arm
    public static final double kG = 1.19;
    public static final double kV = 7.67;
    public static final double kA = 0.19;

}
