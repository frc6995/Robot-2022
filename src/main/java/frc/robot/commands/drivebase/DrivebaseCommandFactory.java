// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.DrivebaseS;

/** Add your docs here. */
public class DrivebaseCommandFactory {

    public static Command createCurvatureDriveC(DoubleSupplier fwdBack, DoubleSupplier turn, DrivebaseS drivebaseS) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                drivebaseS.curvatureDrive(
                    fwdBack.getAsDouble(),
                    turn.getAsDouble()
                );
            },
            interrupted -> {
                drivebaseS.stopAll();
            },
            () -> false
          , drivebaseS);
    }

}
