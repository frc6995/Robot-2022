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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
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

    /*
     * Intake
     * 10, 12
     * 14, 13
     * Battery
     */
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

    public static final int USB_PORT_OPERATOR_CONTROLLER = 1;

    /** Deadband for drive joystick */
    public static final double DRIVEBASE_DEADBAND = 0.03; // anything under 1.5% of full joystick away from 0 should be
                                                          // considered 0

    /** Slew limit for drivebase motors */
    public static final double DRIVEBASE_TURN_SLEW_LIMIT = 2.0; // 0 to 200% in one second. 0 to full in 1/2 second.


    /** Slew limit for drivebase forward back motors */
    public static final double DRIVEBASE_FWD_BACK_SLEW_LIMIT = 1.5; // 0 to 100% in one second.

    /** Drivebase encoder rotations per wheel rotation. */
    public static final double DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION = (28.0/20.0) * (64.0/12.0);

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
     * The track width of the drivebase.
     */
    public static final double DRIVEBASE_TRACKWIDTH = Units.inchesToMeters(22.5);

    /**
     * The feedforward constants for forward-back driving.
     */
    public static final double[] DRIVEBASE_LINEAR_FF = {0.22, 
        1/(473.0) /* volts per NEO RPM*/
        * 60 /*seconds per minute*/
        * DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION
        , 2.0}; // TODO real numbers

    /**
     * The feedforward constants for rotation while driving.
     */
    public static final double[] DRIVEBASE_ANGULAR_FF = {0.22, 1.5/DRIVEBASE_TRACKWIDTH*0.6995, 0.3/DRIVEBASE_TRACKWIDTH*0.6995}; // TODO real numbers

    /**
     * The proportional constant for the drivebase wheel.
     */
    public static final double DRIVEBASE_P = 0.294; // TODO real numbers


    /**
     * The system modeling plant for the drivebase.
     */
    public static final LinearSystem<N2, N2, N2> DRIVEBASE_PLANT = LinearSystemId.identifyDrivetrainSystem(
        DRIVEBASE_LINEAR_FF[1],
        DRIVEBASE_LINEAR_FF[2],
        DRIVEBASE_ANGULAR_FF[1],
        DRIVEBASE_ANGULAR_FF[2],
        DRIVEBASE_TRACKWIDTH
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
    public static final Vector<N7> DRIVEBASE_SIM_ENCODER_STD_DEV = VecBuilder.fill(0, 0, 0.0001, 0.05, 0.05, 0.0001, 0.0001);

    /**
     * Trajectory config for auto.
     */
    public static final TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(3, 2).setEndVelocity(0);
    
   // Shooter Constants
    /** Can ids for front shooter motor */
    public static final int CAN_ID_FRONT_SHOOTER_MOTOR = 40;

    /** Can id for back shooter motor */
    public static final int CAN_ID_BACK_SHOOTER_MOTOR = 41;

    /** Shooter front wheel feed forward */
    public static final double[] SHOOTER_FRONT_FF = { 0.18114, 0.1298, 0.01253 };

    /** Proportional term for front shooter wheel */
    public static final double SHOOTER_FRONT_P = 0.13714;

    /** Shooter back wheel feed forward */
    public static final double[] SHOOTER_BACK_FF = { 0.41933, 0.13317, 0.0042113 };

    /** Proportional term for back shooter wheel */
    public static final double SHOOTER_BACK_P = 0.19475;

    /** Allowable error to still be on target */
    public static final double SHOOTER_PID_ERROR = 200; //rpm

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
    public static final int TURRET_LIMIT_SWITCH_PORT = 3;
  
    /** Counterclockwise limit at which the turret cannot pass */
    public static final double SOFT_LIMIT_FORWARD_RADIAN = Units.degreesToRadians(180 + 110);

    /** Clockwise limit at which the turret cannot pass */
    public static final double SOFT_LIMIT_REVERSE_RADIAN = Units.degreesToRadians(180-110);

    /** Number of degrees of allowable error for turret */
    public static final double TURRET_PID_ERROR = Units.degreesToRadians(1);

    /** Speed at which the turret should go back to home */
    public static final double TURRET_HOMING_SPEED = -0.1;

    /** Deadband for the turret joystick */
    public static final double TURRET_DEADBAND = 0.02;

    /** Feedforward for the turret */
    public static final double[] TURRET_FF = {0.16, 0.56397, 0.030497}; // for velocity in turret radians per second

    /** Proportional term for the turret */
    public static final double TURRET_P = 6;//51.463;

    /** Derivative term for the turret */
    public static final double TURRET_D = 1.9948;

    /** The maximum manual-drive angular velocity of the turret in radians per second */
    public static final double TURRET_MAX_SPEED = 1;

    // Midtake Constants

    /** Can ids for the front midtake motor */
    public static final int CAN_ID_MIDTAKE_FRONT = 23;

    /** Can id for the back midtake motor */
    public static final int CAN_ID_MIDTAKE_BACK = 22;

    /** Port number for the top beam break sensor */
    public static final int BEAM_BREAK_TOP_PORT_NUMBER = 0;

    /** Port number for the bottom beam break sensor */
    public static final int BEAM_BREAK_BOTTOM_PORT_NUMBER = 1;

    /** Proximity threshold on the color sensor for detecting a ball*/
    public static final double COLOR_SENSOR_PROXIMITY_THRESHOLD = 250;

    /** Midtake loading speed */
    public static final double MIDTAKE_LOADING_SPEED = 0.3;

    /** Midtake feeding speed */
    public static final double MIDTAKE_FEEDING_SPEED = 0.8;

    /** Midtake crawling speed, default behavior if no ball is detected. */
    public static final double MIDTAKE_CRAWL_SPEED = 0.1;

    // Limelight Constants

    /** How long to run filter */
    public static final double LIMELIGHT_FILTER_TIME_CONSTANT = 0.1;

    /** Time between each filter measurement */
    public static final double LIMELIGHT_FILTER_PERIOD_CONSTANT = 0.02;
    
    /** Degrees spanned by the diagonal of the camera's field of view */
    public static final double CAMERA_DIAG_FOV_DEGREES = 67.8;

    /** Height of the camera off the ground in meters */
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(38); //37 inches

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
    public static final int DOUBLE_SOLENOID_INTAKE_PORT_EXTEND = 1;

    /** Port number for electromagnet that retracts */
    public static final int DOUBLE_SOLENOID_INTAKE_PORT_RETRACT = 0;

    /** Speed at which the intake motor moves */
    public static final double INTAKE_SPEED = 0.75;

    /** Speed at which the intake ejects balls */
    public static final double INTAKE_EJECT_SPEED = -0.5;

    //Field Constants

    /** The field-relative position of the center of the hub. */
    public static final Pose2d HUB_CENTER_POSE = new Pose2d(
        Units.inchesToMeters(324.0),
        Units.inchesToMeters(162.0),
        Rotation2d.fromDegrees(0));

    /** The length of a vision tape strip in meters. */
    public static final double TAPE_STRIP_LENGTH = Units.inchesToMeters(5);

    /** The height of a vision tape strip in meters. */
    public static final double TAPE_STRIP_HEIGHT = Units.inchesToMeters(2);

    /** The radius of the upper hub vision ring in meters */
    public static final double HUB_RADIUS_METERS = Units.feetToMeters(4.625/2);

    /** The height of the bottom edge of the tape strips off the floor in meters. */
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(104); //8ft 8 inches

    /** The number of tape strips around the vision ring */
    public static final double TAPE_STRIP_COUNT = 16.0;

     //Climber Constants
    /**
     * The proportional constant for the climber
     */
    public static final double CLIMBER_P_CONSTANT = 0;
    /**
     * The integral constant for the climber
     */
    public static final double CLIMBER_I_CONSTANT = 0;
    /**
     * The derivative constant for the climber
     */
    public static final double CLIMBER_D_CONSTANT = 0;
    /**
     * The CAN ID of the climber motor
     */
    public static final int CAN_ID_CLIMBER_MOTOR = 50;
    /**
     * The PID error of the climber
     */
    public static final int CLIMBER_PID_ERROR = 10;
    /**
     * The max amount of rotations extending the climber
     */
    public static final double CLIMBER_FRONT_SOFT_LIMIT_FORWARD = 280.0;
    /**
     * The maximum a mount of rotations retracting the climber
     */
    public static final double CLIMBER_FRONT_SOFT_LIMIT_BACK = 20.0;

    public static final double CLIMBER_FRONT_SOFT_LIMIT_MID = 150;

    public static final double CLIMBER_FRONT_TRANSFER_VOLTS = -3; 

    public static final double CLIMBER_BACK_TRANSFER_VOLTS = 5; 

    /**
     * The current the back motor needs to pull for the climber to detect that it's lifting the robot.
     */
    public static final double CLIMBER_BACK_PULLING_CURRENT = 10;
        /**
     * The max amount of rotations extending the climber
     */
    public static final double CLIMBER_BACK_SOFT_LIMIT_FORWARD = 370.0;
    /**
     * The maximum a mount of rotations retracting the climber
     */
    public static final double CLIMBER_BACK_SOFT_LIMIT_BACK = 0.0;

    public static final double CLIMBER_BACK_SOFT_LIMIT_SHOOTER = 110;

    public static final double CLIMBER_BACK_LIFT_POSITION = 110;

    public static final double CLIMBER_BACK_HOLDING_FF = -1;
    /**
     * The port number of the double solenoid when it folds the climber up
     */
    public static final int DOUBLE_SOLENOID_CLIMBER_FORWARD = 3;
    /**
     * The port number of the double solenoid when it folds the climber down
     */
    public static final int DOUBLE_SOLENOID_CLIMBER_BACK = 2;
    /**
     * The CAN ID of the back climb motor
     */
    public static final int CAN_ID_BACK_CLIMB_MOTOR = 51;
    /**
     * The proportional constant for the back climber
     */
    public static final int BACK_CLIMBER_P_CONSTANT = 0;
    /**
     * The integral constant for the back climber
     */
    public static final int BACK_CLIMBER_I_CONSTANT = 0;
    /**
     * The derivative constant for the back climber
     */
    public static final int BACK_CLIMBER_D_CONSTANT = 0;

    //LED Constants

    /** The roboRIO spark value for solid green LEDs */
    public static final double LED_SOLID_GREEN = 0.77;

    /** The roboRIO spark value for Light Chase pattern */
    public static final double LED_PATTERN_GREEN = 0.01;

    /** The roboRIO spark value for Strobe, Red pattern */
    public static final double LED_PATTERN_RED = -0.11;

    /** The roboRIO spark value for Rainbow, Party Palette pattern */
    public static final double LED_PARTY_MODE = -0.97;

    public static final double LED_GOLD_SOLID = 0.67;

    public static final double LED_GREEN_RAINBOW = -0.91;

    public static final double LED_LARSON_SCANNER = -0.01;

    public static final double LED_CONFETTI_MODE = -0.87;

    public static final int PWM_PORT_LED = 9;

    

}
