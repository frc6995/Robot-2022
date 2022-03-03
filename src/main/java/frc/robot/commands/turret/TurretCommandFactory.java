package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.TurretS;

/** The factory class that creates all Turret commands. */
public class TurretCommandFactory {

    /**
     * Creates a TurretHomingC, which slowly spins the turret towards the homing
     * switch until the turret is homed.
     * 
     * @param turretS the turret subsystem
     * @return the TurretHomingC
     */
    public static Command createTurretHomingC(TurretS turretS) {
        return new FunctionalCommand(
            ()->{},
            turretS::turnHoming,
            interrupted -> {
                turretS.stopMotor();
                if(!interrupted) {
                    turretS.resetEncoder(Math.PI - Constants.SOFT_LIMIT_REVERSE_RADIAN);
                }
            }, turretS::getIsHomed, turretS
        )
        .withName("TurretHomingC");
    }

    /**
     * Creates a TurretTurnC, which uses PID to point the turret to the given angle,
     * where robot intake side is 0.
     * 
     * @param angle   the desired angle in radians counterclockwise from forward
     * @param turretS the turret subsystem
     * @return the TurretTurnC
     */
    public static Command createTurretTurnC(double angle, TurretS turretS) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    turretS.setTurretAngle(new Rotation2d(angle));
                },
                interrupted -> {
                    turretS.stopMotor();
                },
                turretS::isAtTarget,
                turretS)
                        .withName("TurretTurnC");
    }

    /**
     * Creates a TurretFollowC, which uses PID to point the turret to the given
     * angle, where the robot front is 0;
     * 
     * @param angle   a DoubleSupplier for the desired angle
     * @param turretS the turret subsystem
     * @return the TurretTurnC
     */
    public static Command createTurretFollowC(Supplier<Rotation2d> angle, TurretS turretS) {
        return new FunctionalCommand(
            ()->{}, 
            ()->{
                turretS.setTurretAngle(angle.get());
            },
            interrupted -> {
                turretS.stopMotor();
            },
            () -> false,
            turretS
        )
        .withName("TurretFollowC");
    }

    public static Command createTurretVisionC(LimelightS limelightS, TurretS turretS) {
        return new FunctionalCommand(limelightS::ledsOn,
        ()->{
            if (limelightS.hasTarget()) {
                turretS.turnVelocityOpenLoop(limelightS.getFilteredXOffset() * Constants.TURRET_P);
            }
            
        },
        (interrupted)->{
            turretS.stopMotor();
            limelightS.ledsOff();
        }, ()->false, turretS);
    }

    /**
     * Creates a TurretManualC, which turns the turret at a speed given by
     * `speedSupplier` [-1..1].
     * 
     * @param speedSupplier The Supplier for the turret speed
     * @param turretS       the turret subsystem
     * @return the TurretManualC
     */
    public static Command createTurretManualC(DoubleSupplier speedSupplier, TurretS turretS) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    turretS.turnSpeedOpenLoop(speedSupplier.getAsDouble());
                },
                interrupted -> {
                    turretS.stopMotor();
                },
                () -> false,
                turretS)
                        .withName("TurretManualC");
    }

}
