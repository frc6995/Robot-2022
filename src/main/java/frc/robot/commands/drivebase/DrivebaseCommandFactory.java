package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.util.OdometryManager;

/** Factory class to create drive commands */
public class DrivebaseCommandFactory {

  /**
   * Creates a curvature drive command that does not naturally end.
   * 
   * @param fwdBack    the forward/back speed [-1..1]
   * @param turn       the turn tightness [-1..1]
   * @param drivebaseS the drivebase subsystem
   * @return the CurvatureDriveC.
   */
  public static Command createCurvatureDriveC(DoubleSupplier fwdBack, DoubleSupplier turn, DrivebaseS drivebaseS) {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          drivebaseS.curvatureDrive(
              fwdBack.getAsDouble(),
              turn.getAsDouble());
        },
        interrupted -> {
          drivebaseS.stopAll();
        },
        () -> false, drivebaseS)
            .withName("CurvatureDriveC");
  }


  public static Command createTimedDriveC(double power, double time, DrivebaseS drivebaseS) {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          drivebaseS.tankDrive(power, power);
        },
        interrupted -> {
          drivebaseS.tankDrive(0, 0);
        },
        () -> {
          return false;
        },
        drivebaseS)
            .withTimeout(time)
            .withName("DriveTimedC");
  }

  public static Command createRamseteC(Trajectory trajectory,  OdometryManager odometryManager, DrivebaseS drivebaseS) {
    return new RamseteCommand(
        trajectory,
        odometryManager::getEstimatedRobotPose,
        drivebaseS.ramseteController,
        Constants.DRIVEBASE_KINEMATICS,
        drivebaseS::tankDriveVelocity,
        drivebaseS
      );
  }

}
