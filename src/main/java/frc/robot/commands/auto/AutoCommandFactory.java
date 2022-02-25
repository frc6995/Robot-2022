package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.MainCommandFactory;
import frc.robot.commands.drivebase.DrivebaseCommandFactory;
import frc.robot.commands.turret.TurretCommandFactory;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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
    public static Command createTwoBallAutoCG(double targetFrontSpeed, double targetBackSpeed, double targetAngle,
            ShooterS shooterS, IntakeS intakeS, MidtakeS midtakeS, TurretS turretS, DrivebaseS drivebaseS) {
                return new ParallelCommandGroup(
                    // Spins up the front and back shooter motors to the set target speeds
                    new InstantCommand(
                        () -> {
                            shooterS.pidFrontSpeed(targetFrontSpeed);
                            shooterS.pidBackSpeed(targetBackSpeed);
                        },
                        shooterS)
                                .andThen(
                                        new WaitUntilCommand(shooterS::isAtTarget)),
                // Homes the turret
                TurretCommandFactory.createTurretHomingC(turretS)
                        .andThen(
                                TurretCommandFactory.createTurretTurnC(
                                        Constants.SOFT_LIMIT_FORWARD_DEGREE / 2.0, // halfway between 0 and the forward
                                                                                   // limit.
                                        turretS))
                        .andThen(
                                new WaitUntilCommand(turretS::isAtTarget)),
                // Drives at 25% speed for 7 seconds or until the color sensor detects a ball
                // present
                DrivebaseCommandFactory.createTimedDriveC(.25, 7, drivebaseS)
                        .withInterrupt(midtakeS::getColorSensorDetectsBall),
                // Intakes one ball, stores or rejects based on whether the color is correct
                MainCommandFactory.createIntakeIndexCG(intakeS, midtakeS)
                        // Retracts intake
                        .andThen(intakeS::retract, intakeS))
                                // Spins the midtake to feed the shooter
                                .andThen(
                                        new RunCommand(midtakeS::spin, midtakeS)
                                                .withTimeout(10))
                                .andThen(new InstantCommand(
                                        () -> {
                                            midtakeS.stop();
                                            shooterS.stop();
                                        },
                                        midtakeS, shooterS))
                                .withName("Two Ball Auto");
    }

}
