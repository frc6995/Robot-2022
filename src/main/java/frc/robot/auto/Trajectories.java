package frc.robot.auto;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

import static frc.robot.util.NomadMathUtil.*;

import java.util.List;

public class Trajectories {

  public static final TrajectoryConfig getConfig() {
    return new TrajectoryConfig(2.5, 1.25).setEndVelocity(0)
        .addConstraint(
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    Constants.DRIVEBASE_LINEAR_FF_MPS[0],
                    Constants.DRIVEBASE_LINEAR_FF_MPS[1],
                    Constants.DRIVEBASE_LINEAR_FF_MPS[2]),
                Constants.DRIVEBASE_KINEMATICS, 10.0))
        .addConstraint(
            new DifferentialDriveKinematicsConstraint(
                Constants.DRIVEBASE_KINEMATICS,
                4))
        .addConstraint(
            new CentripetalAccelerationConstraint(
                4));

  }

  public static final Pose2d HUB_CENTER_POSE = new Pose2d(
      Units.inchesToMeters(324.0),
      Units.inchesToMeters(162.0),
      Rotation2d.fromDegrees(0));

  public static final Pose2d CARGO_RING_MID_OWN_BALL = new Pose2d(5.066880, 1.895581, new Rotation2d());
  public static final Pose2d CARGO_RING_WALL_OWN_BALL = new Pose2d(7.579211, 0.298761, new Rotation2d());
  public static final Pose2d CARGO_RING_HANGAR_OWN_BALL = new Pose2d(4.960425, 6.217642, new Rotation2d());
  public static final Pose2d TERMINAL_OWN_BALL = new Pose2d(1.085474, 1.129107, new Rotation2d());
  public static final Pose2d MID_BALL_START_POSE = new Pose2d(6.67436, 2.651410, Rotation2d.fromDegrees(-155.055217));
  public static final Pose2d MID_BALL_HUB_SIDE_PICKUP = pickup(
      CARGO_RING_MID_OWN_BALL,
      getDirection(
          new Transform2d(
              HUB_CENTER_POSE, CARGO_RING_MID_OWN_BALL)),
      Units.inchesToMeters(-5));
  public static final Pose2d TERMINAL_BALL_PICKUP = pickup(TERMINAL_OWN_BALL, Rotation2d.fromDegrees(180),
      Units.inchesToMeters(5));

  public static Pose2d pickup(Pose2d target, Rotation2d heading, double stopDistanceMeters) {
    return target.transformBy(
        new Transform2d(
            new Translation2d(-stopDistanceMeters, 0)
                .rotateBy(heading),
            heading));
  }

  public static final Trajectory MID_START_TO_MID_RING = TrajectoryGenerator.generateTrajectory(
      MID_BALL_START_POSE,
      List.of(),
      MID_BALL_HUB_SIDE_PICKUP,
      getConfig());

  public static final Trajectory MID_RING_TO_TERMINAL = TrajectoryGenerator.generateTrajectory(
      MID_BALL_HUB_SIDE_PICKUP,
      List.of(),
      TERMINAL_BALL_PICKUP,
      getConfig());

  public static final Trajectory TERMINAL_TO_MID_RING = TrajectoryGenerator.generateTrajectory(
      TERMINAL_BALL_PICKUP,
      List.of(),
      MID_BALL_HUB_SIDE_PICKUP,
      getConfig().setReversed(true));

}
