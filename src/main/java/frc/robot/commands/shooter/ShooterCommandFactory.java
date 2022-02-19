package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterS;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

/** Factory class to create shoot commands. */
public class ShooterCommandFactory {
    /**
     * Creates a TurretFollowC, which uses PID to point the turret to the given
     * angle, where the homing switch is 0.
     * 
     * @param angle   a DoubleSupplier for the desired angle
     * @param turretS the turret subsystem
     * @return the TurretTurnC
     */
    public static Command createShooterFollowC(DoubleSupplier frontSpeed, DoubleSupplier backSpeed, ShooterS shooterS) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    shooterS.pidFrontSpeed(frontSpeed.getAsDouble());
                    shooterS.pidBackSpeed(backSpeed.getAsDouble());
                },
                interrupted -> {
                    shooterS.setFrontSpeed(0);
                    shooterS.setBackSpeed(0);
                },
                () -> false,
                shooterS)
                        .withName("TurretTurnC");
    }

}
