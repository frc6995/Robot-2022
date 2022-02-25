// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

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
    public static final double DRIVEBASE_DEADBAND = 0.03; // anything under 1.5% of full joystick away from 0 should be considered 0
    public static final double DRIVEBASE_TURN_SLEW_LIMIT = 2.0; // 0 to 200% in one second. 0 to full in 1/2 second.
    public static final double DRIVEBASE_FWD_BACK_SLEW_LIMIT = 1; // 0 to 100% in one second.
    public static final double DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION = 10.71; //TODO get real number
    public static final double DRIVEBASE_WHEEL_DIAMETER = Units.inchesToMeters(6);

    public static final double DRIVEBASE_METERS_PER_WHEEL_ROTATION = Math.PI * DRIVEBASE_WHEEL_DIAMETER;
    public static final double drivbaseEncoderRotationsToMeters(double rotations) {
        return rotations
        * DRIVEBASE_METERS_PER_WHEEL_ROTATION 
        / DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION;
    }
    public static final double[] DRIVEBASE_LINEAR_FF = {0.22, 1.98, 0.2}; // TODO real numbers
    public static final double[] DRIVEBASE_ANGULAR_FF = {0.22, 1.5, 0.3}; // TODO real numbers
    public static final double DRIVEBASE_TRACKWIDTH = 0.69; // TODO real numbers
    public static final LinearSystem<N2, N2, N2> DRIVEBASE_PLANT = LinearSystemId.identifyDrivetrainSystem(
        DRIVEBASE_LINEAR_FF[1],
        DRIVEBASE_LINEAR_FF[2],
        DRIVEBASE_ANGULAR_FF[1],
        DRIVEBASE_ANGULAR_FF[2]
        );
    public static final DifferentialDriveKinematics DRIVEBASE_KINEMATICS = new DifferentialDriveKinematics(DRIVEBASE_TRACKWIDTH);
    public static final DCMotor DRIVEBASE_GEARBOX = DCMotor.getNEO(2);
    public static final Vector<N7> DRIVEBASE_SIM_ENCODER_STD_DEV = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0);
    
    //Shooter Constants
    public static final int CAN_ID_FRONT_SHOOTER_MOTOR = 41;
    public static final int CAN_ID_BACK_SHOOTER_MOTOR = 40;
    public static final double[] SHOOTER_FRONT_FF = {0.16382, 0.12618, 0.0038694};
    public static final double SHOOTER_FRONT_P = 0.13714;
    public static final double[] SHOOTER_BACK_FF = {0.29066, 0.12829, 0.0050724};
    public static final double SHOOTER_BACK_P = 0.19475;
    public static final double SHOOTER_PID_ERROR = 0.5;
    public static final double[] SPEEDS = {2900, 2650, 2600, 2600, 2550, 2600, 3200, 4400};
    public static final double[] DISTANCES = {-25, -18, -15, -13, -9, -4, 6, 20};

    //Turret Constants
    public static final double ENCODER_COUNTS_PER_TURRET_NEO_REVOLUTION = 1;
    public static final double NEO_REVOLUTIONS_PER_TURRET_REVOLUTION = 5.23 * 188 / 18;
    public static final double ENCODER_COUNTS_PER_TURRET_DEGREE = ENCODER_COUNTS_PER_TURRET_NEO_REVOLUTION * NEO_REVOLUTIONS_PER_TURRET_REVOLUTION / 360;
    public static final int CAN_ID_TURRET = 30;
    public static final int TURRET_LIMIT_SWITCH_PORT = 0;
    public static final float SOFT_LIMIT_FORWARD_DEGREE = 110f;
    public static final float SOFT_LIMIT_REVERSE_DEGREE = -110f;
    public static final int TURRET_PID_ERROR = 1;
    public static final double TURRET_HOMING_SPEED = -0.1;
    public static final double TURRET_DEADBAND = 0.02;
    public static final double[] TURRET_FF = {0.39475, 3.5435, 0.19223}; // for velocity in turret rotations per second
    public static final double TURRET_P = 1.0/90.0;
    public static final double TURRET_MAX_SPEED = 1.0;

    //Midtake Constants
    public static final int CAN_ID_MIDTAKE_FRONT = 23;
    public static final int CAN_ID_MIDTAKE_BACK = 22;
    public static final int BEAM_BREAK_TOP_PORT_NUMBER = 1;
    public static final int BEAM_BREAK_BOTTOM_PORT_NUMBER = 2;
    public static final double COLOR_SENSOR_PROXIMITY_THRESHOLD = 250;
    public static final double MIDTAKE_FRONT_MOTOR_SPEED = 0.6995;
    public static final double MIDTAKE_BACK_MOTOR_SPEED = 0.6995;

    //Limelight Constants
    public static final double LIMELIGHT_FILTER_TIME_CONSTANT = 0.1; 
    public static final double LIMELIGHT_FILTER_PERIOD_CONSTANT = 0.02;
    public static final double CAMERA_DIAG_FOV_DEGREES = 67.8; // TODO real numbers
    public static final double CAMERA_HEIGHT_METERS = 0.9398; //37 inches
    public static final double CAMERA_CENTER_OFFSET = Units.inchesToMeters(7); // TODO real numbers
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30); // 30 degrees above horizontal
    public static final int CAMERA_HORIZ_RES = 320;
    public static final int CAMERA_VERT_RES = 240;
  
    //Intake Constants
    public static final int CAN_ID_INTAKE_LEAD_MOTOR = 20;
    public static final int CAN_ID_INTAKE_FOLLOWER_MOTOR = 21;
    public static final int DOUBLE_SOLENOID_INTAKE_EXTEND = 2;
    public static final int DOUBLE_SOLENOID_INTAKE_RETRACT = 3;
    public static final double INTAKE_SPEED = 0.25;
    public static final double INTAKE_EJECT_SPEED = -0.5;

    //Field Constants
    public static final Pose2d HUB_CENTER_POSE = new Pose2d(
        Units.feetToMeters(54.0/2),
        Units.feetToMeters(27.0/2),
        Rotation2d.fromDegrees(0));
    public static final double TAPE_STRIP_LENGTH = Units.inchesToMeters(5);
    public static final double TAPE_STRIP_HEIGHT = Units.inchesToMeters(2);
    public static final double HUB_RADIUS_METERS = Units.feetToMeters(2);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(104); //8ft 8 inches
    public static final double TAPE_STRIP_COUNT = 16.0; // TODO real numbers

}

