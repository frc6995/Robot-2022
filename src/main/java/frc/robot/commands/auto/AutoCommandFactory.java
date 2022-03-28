package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.auto.Trajectories;
import frc.robot.commands.IntakeCommandFactory;
import frc.robot.commands.MainCommandFactory;
import frc.robot.commands.MidtakeCommandFactory;
import frc.robot.commands.drivebase.DrivebaseCommandFactory;
import frc.robot.commands.shooter.ShooterCommandFactory;
import frc.robot.commands.turret.TurretCommandFactory;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.pose.NavigationManager;

public class AutoCommandFactory {
    /**
     * Creates a command group to complete a two ball auto: spins up the shooter,
     * sets the front and
     * back shooter motors, homes the turret, drives, intakes one ball, retracts the
     * intake, and spins midtake to
     * shoot two balls.
     * 
     * @param targetFrontSpeed the target speed of the front shooter motor
     * @param targetBackSpeed  the target speed of the back shooter motor
     * @param targetAngle      the target angle of the turret
     * @param shooterS         the shooter subsystem
     * @param intakeS          the intake subsystem
     * @param midtakeS         the midtake subsystem
     * @param turretS          the turret subsystem
     * @param drivebaseS       the drivebase subsystem
     * @return the two ball auto command group
     */
    public static Command createTwoBallAutoCG(
            ShooterS shooterS, IntakeS intakeS, MidtakeS midtakeS, TurretS turretS, LimelightS limelightS,
            DrivebaseS drivebaseS) {
        return new ParallelCommandGroup(
                ShooterCommandFactory.createShooterFollowC(() -> (1750), () -> (1750), shooterS),
                new ParallelCommandGroup(
                        DrivebaseCommandFactory.createTimedDriveC(.4, 1.5, drivebaseS),
                        MainCommandFactory.createIntakeCG(midtakeS, intakeS)).withTimeout(2.25)
                                .withInterrupt(midtakeS::getIsMidtakeFull)
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeDefaultC(midtakeS)
                                        .withInterrupt(midtakeS::getIsArmed))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeFeedOneC(midtakeS))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeDefaultC(midtakeS)
                                        .withInterrupt(midtakeS::getIsArmed))
                                .andThen(new WaitUntilCommand(shooterS::isAtTarget))
                                .andThen(new WaitCommand(.5))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeFeedC(midtakeS)
                                        .withInterrupt(midtakeS::getIsTopBeamClear)
                                        .withTimeout(2))
                                .andThen(new WaitCommand(0.5)))
                                        .withName("Two Ball Auto");
    }

    public static Command createThreeBallAutoCG(ShooterS shooterS, IntakeS intakeS, MidtakeS midtakeS,
            TurretS turretS,
            LimelightS limelightS, DrivebaseS drivebaseS) {
        return new ParallelCommandGroup(
                ShooterCommandFactory.createShooterFollowC(() -> (1750), () -> (1750), shooterS),
                new ParallelCommandGroup(
                        DrivebaseCommandFactory.createTimedDriveC(.4, 1.75, drivebaseS),
                        MainCommandFactory.createIntakeCG(midtakeS, intakeS))
                                .withInterrupt(midtakeS::getIsMidtakeFull)
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeDefaultC(midtakeS)
                                        .withInterrupt(midtakeS::getIsArmed))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeFeedOneC(midtakeS))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeDefaultC(midtakeS)
                                        .withInterrupt(midtakeS::getIsArmed))
                                .andThen(new WaitUntilCommand(shooterS::isAtTarget))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeFeedC(midtakeS)
                                        .withInterrupt(midtakeS::getIsTopBeamClear)
                                        .withTimeout(2))
                                .andThen(new WaitCommand(0.5))
                                .andThen(DrivebaseCommandFactory.createPivotC(-1.9,
                                        drivebaseS))
                                .andThen(
                                        new ParallelCommandGroup(
                                                DrivebaseCommandFactory
                                                        .createTimedDriveC(
                                                                .4,
                                                                2.5,
                                                                drivebaseS)
                                                        .withInterrupt(midtakeS::getIsTopBeamBroken)
                                                        .andThen(DrivebaseCommandFactory
                                                                .createPivotC(Units
                                                                        .degreesToRadians(
                                                                                -60),
                                                                        drivebaseS)),
                                                MidtakeCommandFactory
                                                        .createMidtakeDefaultC(
                                                                midtakeS)
                                                        .withInterrupt(midtakeS::getIsArmed),
                                                IntakeCommandFactory
                                                        .createIntakeRunC(
                                                                intakeS))
                                                                        .withInterrupt(midtakeS::getIsTopBeamBroken))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeDefaultC(midtakeS)
                                        .withInterrupt(midtakeS::getIsArmed))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeFeedC(midtakeS)
                                        .withTimeout(2)));

    }

    public static Command createRamseteC(DrivebaseS drivebaseS, Trajectory trajectory) {
        return new RamseteCommand(trajectory,
                drivebaseS::getRobotPose,
                drivebaseS.ramseteController, Constants.DRIVEBASE_KINEMATICS,
                drivebaseS::tankDriveVelocity,
                drivebaseS);

    }

  public static Command createFourBallAutoTrajectory(ShooterS shooterS, IntakeS intakeS, MidtakeS midtakeS,
      TurretS turretS,
      LimelightS limelightS, DrivebaseS drivebaseS) {
    return new InstantCommand(
        () -> {
          drivebaseS.resetRobotPose(Trajectories.MID_BALL_START_POSE);
        })
    .andThen(
        new ParallelCommandGroup(
            /*
             * ShooterCommandFactory.createShooterDistanceSpinupC(() -> {
             * return NomadMathUtil
             * .getDistance(new Transform2d(drivebaseS.getRobotPose(),
             * Trajectories.HUB_CENTER_POSE))
             * - Constants.HUB_RADIUS_METERS;
             * }, shooterS),
             */
            ShooterCommandFactory.createShooterFollowC(() -> (1700), () -> (1700), shooterS),
            TurretCommandFactory.createTurretVisionC(limelightS, turretS),
            new ParallelDeadlineGroup(
                DrivebaseCommandFactory.createRamseteC(
                    Trajectories.MID_START_TO_MID_RING, drivebaseS),
                MainCommandFactory.createIntakeCG(midtakeS, intakeS).withInterrupt(midtakeS::getIsMidtakeFull)
                .andThen(MidtakeCommandFactory.createMidtakeArmC(midtakeS))
                    

            )
             // DANGER
            .andThen(MidtakeCommandFactory
                .createMidtakeDefaultC(midtakeS)
                .withInterrupt(midtakeS::getIsArmed))
            .andThen(new WaitUntilCommand(shooterS::isAtTarget))
            .andThen(MidtakeCommandFactory
                .createMidtakeFeedOneC(midtakeS))
            .andThen(MidtakeCommandFactory
                .createMidtakeDefaultC(midtakeS)
                .withInterrupt(midtakeS::getIsArmed))
            .andThen(new WaitUntilCommand(shooterS::isAtTarget))
            .andThen(MidtakeCommandFactory
                .createMidtakeFeedC(midtakeS)
                .withTimeout(0.5))
                        // End Two Ball Auto

                        // Turn a little bit
            .andThen(
                new ParallelDeadlineGroup(
                (
                    DrivebaseCommandFactory.createRamseteC(
                    Trajectories.MID_RING_TO_TERMINAL_PICKUP, drivebaseS)
                    .andThen(DrivebaseCommandFactory.createTimedDriveC(-0.2, 0.5, drivebaseS))
                    .andThen(new WaitCommand(0.25))
                    .andThen(DrivebaseCommandFactory.createRamseteC(Trajectories.TERMINAL_RECEIVE_TO_MID_RING, drivebaseS))
                ),
                MainCommandFactory.createIntakeCG(midtakeS, intakeS)
            )
            )
                        // Intake Second Ball
            .andThen(MidtakeCommandFactory.createMidtakeArmC(midtakeS))

                //
            // .andThen(new WaitCommand(1))
            // Shoot two
            .andThen(MidtakeCommandFactory
                .createMidtakeDefaultC(midtakeS)
                .withInterrupt(midtakeS::getIsArmed))
            .andThen(MidtakeCommandFactory
                .createMidtakeFeedOneC(midtakeS))
            .andThen(MidtakeCommandFactory
                .createMidtakeDefaultC(midtakeS)
                .withInterrupt(midtakeS::getIsArmed))
            .andThen(new WaitUntilCommand(shooterS::isAtTarget))
            // .andThen(new WaitCommand(.5))
            .andThen(MidtakeCommandFactory
                .createMidtakeFeedC(midtakeS)
                .withInterrupt(midtakeS::getIsTopBeamClear)
                .withTimeout(2))
            .andThen(new WaitCommand(0.5))
        )
    );
  }

    public static Command createFourBallAuto(ShooterS shooterS, IntakeS intakeS, MidtakeS midtakeS, TurretS turretS,
            LimelightS limelightS, DrivebaseS drivebaseS) {
        return new ParallelCommandGroup(
                ShooterCommandFactory.createShooterFollowC(() -> (1750), () -> (1750), shooterS),
                new ParallelCommandGroup(
                        DrivebaseCommandFactory.createTimedDriveC(.4, 1.5, drivebaseS),
                        MainCommandFactory.createIntakeCG(midtakeS, intakeS)).withTimeout(2.25)
                                .withInterrupt(midtakeS::getIsMidtakeFull)
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeDefaultC(midtakeS)
                                        .withInterrupt(midtakeS::getIsArmed))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeFeedOneC(midtakeS))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeDefaultC(midtakeS)
                                        .withInterrupt(midtakeS::getIsArmed))
                                .andThen(new WaitUntilCommand(shooterS::isAtTarget))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeFeedC(midtakeS)
                                        .withInterrupt(midtakeS::getIsTopBeamClear)
                                        .withTimeout(2))
                                .andThen(new WaitCommand(0.5))
                                // End Two Ball Auto

                                // Turn a little bit
                                .andThen(DrivebaseCommandFactory.createPivotC(
                                        Units.degreesToRadians(-26.75),
                                        drivebaseS).withTimeout(1.75)
                                        .alongWith(TurretCommandFactory
                                                .createTurretFollowC(
                                                        () -> new Rotation2d(
                                                                Units.degreesToRadians(
                                                                        198)),
                                                        turretS)
                                                .withTimeout(1.5)))

                                // Drive and Pick up one ball
                                .andThen(new ParallelCommandGroup(
                                        DrivebaseCommandFactory
                                                .createTimedDriveC(.4,
                                                        2.9,
                                                        drivebaseS),
                                        MainCommandFactory.createIntakeCG(
                                                midtakeS, intakeS)
                                                .withInterrupt(midtakeS::getIsBottomBeamBroken))
                                                        .withTimeout(3.4))

                                // Intake Second Ball
                                .andThen(MainCommandFactory
                                        .createIntakeCG(midtakeS, intakeS)
                                        .withTimeout(2.9)

                                        // Drive Back
                                        .alongWith(DrivebaseCommandFactory
                                                .createTimedDriveC(-0.4,
                                                        2.9,
                                                        drivebaseS))
                                // .andThen(new WaitCommand(1))
                                )
                                // Shoot two
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeDefaultC(midtakeS)
                                        .withInterrupt(midtakeS::getIsArmed))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeFeedOneC(midtakeS))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeDefaultC(midtakeS)
                                        .withInterrupt(midtakeS::getIsArmed))
                                .andThen(new WaitUntilCommand(shooterS::isAtTarget))
                                // .andThen(new WaitCommand(.5))
                                .andThen(MidtakeCommandFactory
                                        .createMidtakeFeedC(midtakeS)
                                        .withInterrupt(midtakeS::getIsTopBeamClear)
                                        .withTimeout(2))
                                .andThen(new WaitCommand(0.5))

        )
                .withName("Four Ball Auto");
    }
}
