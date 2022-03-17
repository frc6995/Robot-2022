package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.MainCommandFactory;
import frc.robot.commands.MidtakeCommandFactory;
import frc.robot.commands.drivebase.DrivebaseCommandFactory;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;

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
            ShooterS shooterS, IntakeS intakeS, MidtakeS midtakeS, TurretS turretS, LimelightS limelightS, DrivebaseS drivebaseS) {
        return new ParallelCommandGroup(
            MainCommandFactory.createVisionSpinupAndAimC(limelightS, turretS, shooterS),
            new ParallelCommandGroup(
                        DrivebaseCommandFactory.createTimedDriveC(.3, 2.5, drivebaseS),
                        new ProxyScheduleCommand(
                                MainCommandFactory.createIntakeCG(midtakeS, intakeS)
                        )
            ).withInterrupt(midtakeS::getIsMidtakeFull)
            .andThen(new WaitCommand(1))
            .andThen(new WaitCommand(3).withInterrupt(midtakeS::getIsArmed))
            .andThen(new WaitCommand(1))
            .andThen(new ProxyScheduleCommand(MidtakeCommandFactory.createMidtakeFeedOneC(midtakeS)))
            .andThen(new WaitCommand(3).withInterrupt(()->(shooterS.isAtTarget() && midtakeS.getIsArmed())))
            .andThen(new ProxyScheduleCommand(MidtakeCommandFactory.createMidtakeFeedOneC(midtakeS)))
        )
        .withName("Two Ball Auto");
    }

    public static Command createThreeBallAutoCG(ShooterS shooterS, IntakeS intakeS, MidtakeS midtakeS, TurretS turretS, LimelightS limelightS, DrivebaseS drivebaseS) {
        return new ParallelCommandGroup(
            MainCommandFactory.createVisionSpinupAndAimC(limelightS, turretS, shooterS),
            new ParallelCommandGroup(
                        DrivebaseCommandFactory.createTimedDriveC(.4, 2.5, drivebaseS),
                        new ProxyScheduleCommand(
                                MainCommandFactory.createIntakeCG(midtakeS, intakeS)
                        )
            ).withInterrupt(midtakeS::getIsMidtakeFull)
            .andThen(MidtakeCommandFactory.createMidtakeArmC(midtakeS))
            .andThen(MidtakeCommandFactory.createMidtakeFeedOneC(midtakeS))
            .andThen(MidtakeCommandFactory.createMidtakeArmC(midtakeS))
            .andThen(new WaitUntilCommand(shooterS::isAtTarget))
            .andThen(MidtakeCommandFactory.createMidtakeFeedOneC(midtakeS))
            .andThen(new WaitCommand(0.5))
            .andThen(DrivebaseCommandFactory.createPivotC(2.134, drivebaseS))
            .andThen(
                new ParallelDeadlineGroup(
                    DrivebaseCommandFactory.createTimedDriveC(.4, 5, drivebaseS),
                    MidtakeCommandFactory.createMidtakeLoadC(midtakeS)
                ).withInterrupt(midtakeS::getIsTopBeamBroken)
            )
            .andThen(MidtakeCommandFactory.createMidtakeArmC(midtakeS))
            .andThen(MidtakeCommandFactory.createMidtakeFeedOneC(midtakeS))
        );

    }
    

    public static Command createRamseteC(DrivebaseS drivebaseS, Trajectory trajectory) {
        return new RamseteCommand(trajectory,
        drivebaseS::getRobotPose,
        drivebaseS.ramseteController, Constants.DRIVEBASE_KINEMATICS, drivebaseS::tankDriveVelocity, drivebaseS);
    
    }

}
