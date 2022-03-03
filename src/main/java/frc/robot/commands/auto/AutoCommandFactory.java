package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommandFactory;
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
        return new InstantCommand(
                ()->{
                        midtakeS.resetCargoCount(0, 1);
                        drivebaseS.resetRobotPose(drivebaseS.START_POSE);
                }
        ).andThen(
                new ParallelDeadlineGroup(
                        new ParallelCommandGroup(
                                // Spins up the front and back shooter motors to the set target speeds
                                // Drives at 25% speed for 7 seconds or until the color sensor detects a ball
                                // present
                                DrivebaseCommandFactory.createTimedDriveC(.1, 2.5, drivebaseS)
                                        .withInterrupt(midtakeS::getIsBottomBeamBroken),
                                new ParallelDeadlineGroup(
                                        IntakeCommandFactory.createIntakeDeployAndRunCG(intakeS)
                                        .withInterrupt(midtakeS::getIsBottomBeamBroken)
                                        .andThen(IntakeCommandFactory.createIntakeStopAndRetractCG(intakeS))
                                        .andThen(new WaitCommand(0.5)),
                                        MidtakeCommandFactory.createMidtakeIndexCG(midtakeS)
                                )                       
                                // Spins the midtake to feed the shooter
                                .andThen(
                                        MidtakeCommandFactory.createMidtakeFeedOneC(midtakeS))                        
                                .andThen(
                                        MidtakeCommandFactory.createMidtakeFeedOneC(midtakeS))
                        ), 
                        MainCommandFactory.createVisionSpinupAndAimC(limelightS, turretS, shooterS)
                )
                .andThen(
                        new InstantCommand(
                                () -> {
                                        shooterS.stop();
                                        midtakeS.stop();
                                },
                                midtakeS, shooterS
                        )
                )
        )        
        .withName("Two Ball Auto");
    }

}
