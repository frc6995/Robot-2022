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
    //Drivebase Constants
    public static final int CAN_ID_FRONT_LEFT_DRIVE_MOTOR = 13;
    public static final int CAN_ID_FRONT_RIGHT_DRIVE_MOTOR = 14;
    public static final int CAN_ID_BACK_LEFT_DRIVE_MOTOR = 12;
    public static final int CAN_ID_BACK_RIGHT_DRIVE_MOTOR = 10;
    public static final int USB_PORT_DRIVER_CONTROLLER = 0;
    public static final double DRIVEBASE_DEADBAND = 0.015; // anything under 1.5% of full joystick away from 0 should be considered 0
    public static final double DRIVEBASE_TURN_SLEW_LIMIT = 2.0; // 0 to 200% in one second. 0 to full in 1/2 second.
    public static final double DRIVEBASE_FWD_BACK_SLEW_LIMIT = 1; // 0 to 100% in one second.
    
    //Midtake Constants
    public static final int CAN_ID_MIDTAKE_FRONT = 21;
    public static final int CAN_ID_MIDTAKE_BACK = 22;
    public static final int BEAM_BREAK_TOP_PORT_NUMBER = 1;
    public static final int BEAM_BREAK_BOTTOM_PORT_NUMBER = 2;
    public static final double MIDTAKE_FRONT_MOTOR_SPEED = 0.6995;
    public static final double MIDTAKE_BACK_MOTOR_SPEED = 0.6995;
}

