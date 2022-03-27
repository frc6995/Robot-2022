package frc.robot.commands.drivebase;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseS;

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
        () -> false,
        drivebaseS)
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

  public static Command createRamseteC(Trajectory trajectory, DrivebaseS drivebaseS) {
    return new RamseteCommand(
        trajectory,
        drivebaseS::getRobotPose,
        drivebaseS.ramseteController,
        Constants.DRIVEBASE_KINEMATICS,
        drivebaseS::tankDriveVelocity,
        drivebaseS
      ).andThen(()->drivebaseS.tankDrive(0, 0), drivebaseS);
  }

  public static Command createPivotC(double angleDelta, DrivebaseS drivebaseS) {
    return new Command() {

      double initialAngle = drivebaseS.getEstimatedHeading().getRadians();
      double targetAngle = initialAngle + angleDelta;

      @Override
          public void initialize() {
              
              initialAngle = drivebaseS.getEstimatedHeading().getRadians();
              //drivebaseS.angularPID.reset(initialAngle);
              SmartDashboard.putNumber("pivotInitialAngle", initialAngle);
              SmartDashboard.putNumber("pivotTargetAngle", targetAngle);
          }
      @Override
      public void execute() {
        drivebaseS.pivot(targetAngle);
      }

      @Override
      public boolean isFinished() {
          return drivebaseS.isPivotAtTarget();
      }
      
      public void end(boolean interrupted) {
        drivebaseS.stopAll();
      }

      @Override
      public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return Set.of(drivebaseS);
      }

    };
  }

}
