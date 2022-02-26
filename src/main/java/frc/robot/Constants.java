package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
    public static final double DRIVEBASE_DEADBAND = 0.03; // anything under 1.5% of full joystick away from 0 should be
                                                          // considered 0

    /** Slew limit for drivebase motors */
    public static final double DRIVEBASE_TURN_SLEW_LIMIT = 2.0; // 0 to 200% in one second. 0 to full in 1/2 second.


    /** Slew limit for drivebase forward back motors */
    public static final double DRIVEBASE_FWD_BACK_SLEW_LIMIT = 1; // 0 to 100% in one second.

    /** Drivebase encoder rotations per wheel rotation. */
    public static final double DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION = 28.0/20.0 * 64.0/11.0;

    /** The maximum teleop velocity of the drivebase in meters per second. */
    public static final double DRIVEBASE_MAX_WHEEL_VELOCITY_MPS = 3.0;

    /** The wheel diameter of the drivebase in meters. Equivalent to 6 inches.*/
    public static final double DRIVEBASE_WHEEL_DIAMETER = Units.inchesToMeters(6);

    /** The circumference of the drivebase wheel. */
    public static final double DRIVEBASE_METERS_PER_WHEEL_ROTATION = Math.PI * DRIVEBASE_WHEEL_DIAMETER;

    /**
     * Converts drivebase encoder rotations to meters traveled by the drivebase wheel.
     * @param rotations The drivebase encoder rotations.
     * @return meters traveled by the drivebase wheel.
     */
    public static final double drivebaseEncoderRotationsToMeters(double rotations) {
        return rotations
        * DRIVEBASE_METERS_PER_WHEEL_ROTATION 
        / DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION;
    }

    /**
     * The feedforward constants for forward-back driving.
     */
    public static final double[] DRIVEBASE_LINEAR_FF = {0.22, 1.98, 0.2}; // TODO real numbers

    /**
     * The feedforward constants for rotation while driving.
     */
    public static final double[] DRIVEBASE_ANGULAR_FF = {0.22, 1.5, 0.3}; // TODO real numbers

    /**
     * The track width of the drivebase.
     */
    public static final double DRIVEBASE_TRACKWIDTH = Units.inchesToMeters(22.5);

    /**
     * The system modeling plant for the drivebase.
     */
    public static final LinearSystem<N2, N2, N2> DRIVEBASE_PLANT = LinearSystemId.identifyDrivetrainSystem(
        DRIVEBASE_LINEAR_FF[1],
        DRIVEBASE_LINEAR_FF[2],
        DRIVEBASE_ANGULAR_FF[1],
        DRIVEBASE_ANGULAR_FF[2]
        );
    
    /** The DifferentialDriveKinematics for the drivebase. */
    public static final DifferentialDriveKinematics DRIVEBASE_KINEMATICS = new DifferentialDriveKinematics(DRIVEBASE_TRACKWIDTH);

    /**
     * The gearbox configuration for the drivebase.
     */
    public static final DCMotor DRIVEBASE_GEARBOX = DCMotor.getNEO(2);

    /**
     * Standard deviations for noise in simulation encoders.
     */
    public static final Vector<N7> DRIVEBASE_SIM_ENCODER_STD_DEV = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0);
    
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

    /** Shooter speed of [frontWheel, backWheel] in order. */
    public static final double[][] SHOOTER_SPEEDS = { {2900, 2900}, {2650, 2650}, {2600, 2600}, {2600, 2600}, {2550, 2550} , {2600, 2600}, {3200, 3200}, {4400, 4400} };

    /** Shooter distances that correspond to speeds */
    public static final double[] SHOOTER_DISTANCES = { -25, -18, -15, -13, -9, -4, 6, 20 };

    // Turret Constants

    /** Number of encoder counts per turret motor revolution */
    public static final double ENCODER_COUNTS_PER_TURRET_NEO_REVOLUTION = 1;

    /** Neo revolutions per revolution of turret */
    public static final double NEO_REVOLUTIONS_PER_TURRET_REVOLUTION = 5.23 * 188 / 18;

    /** Encoder counts per turret degree */
    public static final double ENCODER_COUNTS_PER_TURRET_DEGREE = ENCODER_COUNTS_PER_TURRET_NEO_REVOLUTION
            * NEO_REVOLUTIONS_PER_TURRET_REVOLUTION / 360;

    /** Can id of turret motor */
    public static final int CAN_ID_TURRET = 30;

    /** Port that limit switch goes into */
    public static final int TURRET_LIMIT_SWITCH_PORT = 0;
  
    /** Counterclockwise limit at which the turret cannot pass */
    public static final double SOFT_LIMIT_FORWARD_RADIAN = Units.degreesToRadians(110);

    /** Clockwise limit at which the turret cannot pass */
    public static final double SOFT_LIMIT_REVERSE_RADIAN = Units.degreesToRadians(-110);

    /** Number of degrees of allowable error for turret */
    public static final int TURRET_PID_ERROR = 1;

    /** Speed at which the turret should go back to home */
    public static final double TURRET_HOMING_SPEED = -0.1;

    /** Deadband for the turret joystick */
    public static final double TURRET_DEADBAND = 0.02;

    /** Feedforward for the turret */
    public static final double[] TURRET_FF = {0.39475, Units.radiansToRotations(3.5435), Units.radiansToRotations(0.19223)}; // for velocity in turret rotations per second

    /** Proportional term for the turret */
    public static final double TURRET_P = 2.0/Math.PI;

    /** The maximum manual-drive angular velocity of the turret in rotations per second */
    public static final double TURRET_MAX_SPEED = 1.0;


    // Midtake Constants

    /** Can ids for the front midtake motor */
    public static final int CAN_ID_MIDTAKE_FRONT = 23;

    /** Can id for the back midtake motor */
    public static final int CAN_ID_MIDTAKE_BACK = 22;

    /** Port number for the top beam break sensor */
    public static final int BEAM_BREAK_TOP_PORT_NUMBER = 1;

    /** Port number for the bottom beam break sensor */
    public static final int BEAM_BREAK_BOTTOM_PORT_NUMBER = 2;

    /** Proximity threshold on the color sensor for detecting a ball*/
    public static final double COLOR_SENSOR_PROXIMITY_THRESHOLD = 250;

    /** Midtake front motor speed */
    public static final double MIDTAKE_FRONT_MOTOR_SPEED = 0.6995;

    /** Midtake back motor speed */
    public static final double MIDTAKE_BACK_MOTOR_SPEED = 0.6995;

    // Limelight Constants
    /** Degrees spanned by the diagonal of the camera's field of view */
    public static final double CAMERA_DIAG_FOV_DEGREES = 67.8;

    /** Height of the camera off the ground in meters */
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(37); //37 inches

    /** Offset of the camera from the center of the turret in meters */
    public static final double CAMERA_CENTER_OFFSET = Units.inchesToMeters(9.5); 

    /** Angle of elevation of the camera in radians */
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30); // 30 degrees above horizontal

    /** Horizontal resolution of the camera in pixels */
    public static final int CAMERA_HORIZ_RES = 320;

    /** Vertical resolution of the camera in pixels */
    public static final int CAMERA_VERT_RES = 240;
  
    //Intake Constants

    /** Can id of the leader intake motor */
    public static final int CAN_ID_INTAKE_LEAD_MOTOR = 20;

    /** Can id of the follower intake motor */
    public static final int CAN_ID_INTAKE_FOLLOWER_MOTOR = 21;

    /** Port number for each electromagnet that extends */
    public static final int DOUBLE_SOLENOID_INTAKE_PORT_EXTEND = 2;

    /** Port number for electromagnet that retracts */
    public static final int DOUBLE_SOLENOID_INTAKE_PORT_RETRACT = 3;

    /** Speed at which the intake motor moves */
    public static final double INTAKE_SPEED = 0.25;

    /** Speed at which the intake ejects balls */
    public static final double INTAKE_EJECT_SPEED = -0.5;

    //Field Constants

    /** The field-relative position of the center of the hub. */
    public static final Pose2d HUB_CENTER_POSE = new Pose2d(
        Units.feetToMeters(54.0/2),
        Units.feetToMeters(27.0/2),
        Rotation2d.fromDegrees(0));

    /** The length of a vision tape strip in meters. */
    public static final double TAPE_STRIP_LENGTH = Units.inchesToMeters(5);

    /** The height of a vision tape strip in meters. */
    public static final double TAPE_STRIP_HEIGHT = Units.inchesToMeters(2);

    /** The radius of the upper hub vision ring in meters */
    public static final double HUB_RADIUS_METERS = Units.feetToMeters(2);

    /** The height of the bottom edge of the tape strips off the floor in meters. */
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(104); //8ft 8 inches

    /** The number of tape strips around the vision ring */
    public static final double TAPE_STRIP_COUNT = 16.0;

}
