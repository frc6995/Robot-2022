// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterS;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

/** Add your docs here. */
public class ShooterCommandFactory {
    /**
     * Creates a ShooterFollowC, which uses PID to run the front and back wheels to the given speeds.
     * @param frontSpeed a DoubleSupplier for the desired front wheel speed
     * @param backSpeed a DoubleSupplier for the desired back wheel speed
     * @param shooterS the shooter subsystem
     * @return the ShooterFollowC
     */
    public static Command createShooterFollowC(DoubleSupplier frontSpeed, DoubleSupplier backSpeed, ShooterS shooterS) {
        return new FunctionalCommand(
            ()->{}, 
            ()->{
                shooterS.pidFrontSpeed(frontSpeed.getAsDouble());
                shooterS.pidBackSpeed(backSpeed.getAsDouble());
            },
            interrupted -> {
                shooterS.setFrontSpeed(0);
                shooterS.setBackSpeed(0);
            },
            () -> false,
            shooterS
        )
        .withName("ShooterFollowC");
    }


}
