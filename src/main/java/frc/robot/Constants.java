package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /** Can ids for the front left drive motor */
    public static final int CAN_ID_FRONT_LEFT_DRIVE_MOTOR = 13;

    /** Can ids for the front right drive motor */
    public static final int CAN_ID_FRONT_RIGHT_DRIVE_MOTOR = 14;

    /** Can ids for the back left drive motor */
    public static final int CAN_ID_BACK_LEFT_DRIVE_MOTOR = 12;

    /** Can ids for the back right drive motor */
    public static final int CAN_ID_BACK_RIGHT_DRIVE_MOTOR = 10;

    /** USB port of xbox controller for driver */
    public static final int USB_PORT_DRIVER_CONTROLLER = 0;

    /** Deadband for drive joystick */
    public static final double DRIVEBASE_DEADBAND = 0.02; // anything under 1.5% of full joystick away from 0 should be
                                                          // considered 0

    /** Slew limit for drivebase motors */
    public static final double DRIVEBASE_TURN_SLEW_LIMIT = 2.0; // 0 to 200% in one second. 0 to full in 1/2 second.

    /** SLew limit for drivebase forward back motors */
    public static final double DRIVEBASE_FWD_BACK_SLEW_LIMIT = 1; // 0 to 100% in one second.

    // Shooter Constants
    /** Can ids for front shooter motor */
    public static final int CAN_ID_FRONT_SHOOTER_MOTOR = 41;

    /** Can id for back shooter motor */
    public static final int CAN_ID_BACK_SHOOTER_MOTOR = 40;

    /** Shooter front wheel feed forward */
    public static final double[] SHOOTER_FRONT_FF = { 0.16382, 0.12618, 0.0038694 };

    /** Proportional term for front shooter wheel */
    public static final double SHOOTER_FRONT_P = 0.13714;

    /** Shooter back wheel feed forward */
    public static final double[] SHOOTER_BACK_FF = { 0.29066, 0.12829, 0.0050724 };

    /** Proportional term for back shooter wheel */
    public static final double SHOOTER_BACK_P = 0.19475;

    /** Allowable error to still be on target */
    public static final double SHOOTER_PID_ERROR = 0.5;

    /** Shooter speed of motors in order */
    public static final double[] SPEEDS = { 2900, 2650, 2600, 2600, 2550, 2600, 3200, 4400 };

    /** Shooter distances that correspond to speeds */
    public static final double[] DISTANCES = { -25, -18, -15, -13, -9, -4, 6, 20 };

    // Turret Constants

    /** Number of encoder counts per turret motor revolution */
    public static final double ENCODER_COUNTS_PER_TURRET_NEO_REVOLUTION = 1;

    /** Neo revolutions per revolution of turret */
    public static final double NEO_REVOLUTIONS_PER_TURRET_REVOLUTION = 5.23 * 188 / 18;

    /** equation for encoder counts per turret degree */
    public static final double ENCODER_COUNTS_PER_TURRET_DEGREE = ENCODER_COUNTS_PER_TURRET_NEO_REVOLUTION
            * NEO_REVOLUTIONS_PER_TURRET_REVOLUTION / 360;

    /** Can id of turret motor */
    public static final int CAN_ID_TURRET = 30;

    /** Port that limit switch goes into */
    public static final int TURRET_LIMIT_SWITCH_PORT = 0;

    /** limit at which the turret cannot pass */
    public static final float SOFT_LIMIT_FORWARD_DEGREE = 720.0f;

    /** limit at which the turret cannot pass */
    public static final float SOFT_LIMIT_REVERSE_DEGREE = 0.0f;

    /** Number of degrees of allowable error for turret */
    public static final int TURRET_PID_ERROR = 1;

    /** Speed at which the turret should go back to home */
    public static final double TURRET_HOMING_SPEED = -0.1;

    /** Deadband for the turret joystick */
    public static final double TURRET_DEADBAND = 0.02;

    /** Turret feed forward */
    public static final double[] TURRET_FF = { 0.39475, 3.5435, 0.19223 }; // for velocity in turret rotations per
                                                                           // second
    /** Proportional term for turret motor */
    public static final double TURRET_P = 1.0 / 90.0;

    // Midtake Constants

    /** Can ids for the front midtake motor */
    public static final int CAN_ID_MIDTAKE_FRONT = 23;

    /** Can id for the back midtake motor */
    public static final int CAN_ID_MIDTAKE_BACK = 22;

    /** Port numbers for the top beam break sensor */
    public static final int BEAM_BREAK_TOP_PORT_NUMBER = 1;

    /** Port numbers for the bottom beam break sensor */
    public static final int BEAM_BREAK_BOTTOM_PORT_NUMBER = 2;

    /** Says how close the threshold will be to find the correct color */
    public static final double COLOR_SENSOR_PROXIMITY_THRESHOLD = 250;

    /** speed at which the front midtake motors spin */
    public static final double MIDTAKE_FRONT_MOTOR_SPEED = 0.6995;

    /** speed at which the midtake back motor spins */
    public static final double MIDTAKE_BACK_MOTOR_SPEED = 0.6995;

    // Limelight Constants

    /** How long to run filter */
    public static final double LIMELIGHT_FILTER_TIME_CONSTANT = 0.1;

    /** Time between each filter measurement */
    public static final double LIMELIGHT_FILTER_PERIOD_CONSTANT = 0.02;

    /** Camera height from floor */
    public static final double CAMERA_HEIGHT_METERS = 0.9398; // 37 inches

    /** Target height from floor */
    public static final double TARGET_HEIGHT_METERS = 2.6416; // 8ft 8 inches

    /** angle of camera from horizontal */
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30); // 30 degrees above horizontal

    // Intake Constants

    /** Can ids of the leader intake motor */
    public static final int CAN_ID_INTAKE_LEAD_MOTOR = 20;

    /** Can id of the follower intake motor */
    public static final int CAN_ID_INTAKE_FOLLOWER_MOTOR = 21;

    /** Port number for each electromagnet that extends */
    public static final int DOUBLE_SOLENOID_INTAKE_PORT_EXTEND = 2;

    /** port number for electromagnet that retracts */
    public static final int DOUBLE_SOLENOID_INTAKE_PORT_RETRACT = 3;

    /** Speed at which the intake motor moves */
    public static final double INTAKE_SPEED = 0.25;

    /** Speed at which the intake ejects balls */
    public static final double INTAKE_EJECT_SPEED = -0.5;

}
