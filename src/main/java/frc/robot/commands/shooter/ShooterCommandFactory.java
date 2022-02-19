// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.LightS;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

/** Add your docs here. */
public class ShooterCommandFactory {
    private static LightS lightS = new LightS();
    /**
     * Creates a TurretFollowC, which uses PID to point the turret to the given angle, where the homing switch is 0.
     * @param angle a DoubleSupplier for the desired angle
     * @param turretS the turret subsystem
     * @return the TurretTurnC
     */
    public static Command createShooterFollowC(DoubleSupplier frontSpeed, DoubleSupplier backSpeed, ShooterS shooterS) {
        return new FunctionalCommand(
            ()->{
                lightS.requestState(LightS.states.Shooting);
            },
            ()->{
                shooterS.pidFrontSpeed(frontSpeed.getAsDouble());
                shooterS.pidBackSpeed(backSpeed.getAsDouble());
            },
            interrupted -> {
                shooterS.setFrontSpeed(0);
                shooterS.setBackSpeed(0);
                lightS.requestState(LightS.states.Default);
            },
            () -> false,
            shooterS
        )
        .withName("TurretTurnC");
    }


}
