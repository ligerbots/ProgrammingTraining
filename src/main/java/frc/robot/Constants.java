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
    // Following four CAN IDs are for the drivetrain subsystem
    public static final int LEADER_LEFT_CAN_ID = 12;
    public static final int LEADER_RIGHT_CAN_ID = 8;
    public static final int FOLLOWER_LEFT_CAN_ID = 9; 
    public static final int FOLLOWER_RIGHT_CAN_ID = 11;

    public static final double DRIVETRAIN_KP = 50.0; // waited to be tuned
    public static final double DRIVETRAIN_KI = 0.0;
    public static final double DRIVETRAIN_KD = 0.0; 
    public static final double DRIVETRAIN_KF = 0.0;
}
