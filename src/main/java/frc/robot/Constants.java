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
    public static final int CAN_ID_FRONT_LEFT_DRIVE_MOTOR = 13;
    public static final int CAN_ID_FRONT_RIGHT_DRIVE_MOTOR = 14;
    public static final int CAN_ID_BACK_LEFT_DRIVE_MOTOR = 12;
    public static final int CAN_ID_BACK_RIGHT_DRIVE_MOTOR = 10;
    public static final int USB_PORT_DRIVER_CONTROLLER = 0;
    public static final double DRIVEBASE_DEADBAND = 0.02; // anything under 1.5% of full joystick away from 0 should be considered 0
    public static final double DRIVEBASE_TURN_SLEW_LIMIT = 2.0; // 0 to 200% in one second. 0 to full in 1/2 second.
    public static final double DRIVEBASE_FWD_BACK_SLEW_LIMIT = 1; // 0 to 100% in one second.
    
    public static final double[] SHOOTER_FRONT_FF = {0.16382, 0.12618, 0.0038694};
    public static final double SHOOTER_FRONT_P = 0.13714;
    public static final double[] SHOOTER_BACK_FF = {0.29066, 0.12829, 0.0050724};
    public static final double SHOOTER_BACK_P = 0.19475;
    public static final int SHOOTER_PID_ERROR = 100;
    public static final double[] SPEEDS = {2900, 2650, 2600, 2600, 2550, 2600, 3200, 4400};
    public static final double[] DISTANCES = {-25, -18, -15, -13, -9, -4, 6, 20};
}

