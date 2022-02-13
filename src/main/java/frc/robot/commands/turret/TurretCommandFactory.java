// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.TurretS;

/** 
 * The factory class that creates all Turret commands.
 */
public class TurretCommandFactory {

    /**
     * Creates a TurretHomingC, which slowly spins the turret towards the homing switch until the turret is homed.
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
                    turretS.resetEncoder();
                }
            }, turretS::getIsHomed, turretS
        )
        .withName("TurretHomingC");
    }

    /**
     * Creates a TurretTurnC, which uses PID to point the turret to the given angle, where the homing switch is 0.
     * @param angle the desired angle
     * @param turretS the turret subsystem
     * @return the TurretTurnC
     */
    public static Command createTurretTurnC(double angle, TurretS turretS) {
        return new FunctionalCommand(
            ()->{}, 
            ()->{
                turretS.setTurretAngle(angle);
            },
            interrupted -> {
                turretS.stopMotor();
            },
            turretS::isAtTarget,
            turretS
        )
        .withName("TurretTurnC");
    }

    /**
     * Creates a TurretFollowC, which uses PID to point the turret to the given angle, where the homing switch is 0.
     * @param angle a DoubleSupplier for the desired angle
     * @param turretS the turret subsystem
     * @return the TurretTurnC
     */
    public static Command createTurretFollowC(DoubleSupplier angle, TurretS turretS) {
        return new FunctionalCommand(
            ()->{}, 
            ()->{
                turretS.setTurretAngle(angle.getAsDouble());
            },
            interrupted -> {
                turretS.stopMotor();
            },
            () -> false,
            turretS
        )
        .withName("TurretTurnC");
    }

    /**
     * Creates a TurretManualC, which turns the turret at a speed given by `speedSupplier` [-1..1].
     * @param speedSupplier The Supplier for the turret speed
     * @param turretS the turret subsystem
     * @return the TurretManualC
     */
    public static Command createTurretManualC(DoubleSupplier speedSupplier, TurretS turretS) {
        return new FunctionalCommand(
            ()->{},
            ()->{
                turretS.turnSpeed(speedSupplier.getAsDouble());
            },
            interrupted -> {
                turretS.stopMotor();
            },
            () -> false,
            turretS
        )
        .withName("TurretManualC");
    }




}
