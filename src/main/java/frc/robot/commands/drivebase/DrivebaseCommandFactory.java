// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.DrivebaseS;

/** Factory class to create drive commands */
public class DrivebaseCommandFactory {

    /**
     * Creates a curvature drive command that does not naturally end.
     * @param fwdBack the forward/back speed [-1..1]
     * @param turn the turn tightness [-1..1]
     * @param drivebaseS the drivebase subsystem
     * @return the CurvatureDriveC.
     */
    public static Command createCurvatureDriveC(DoubleSupplier fwdBack, DoubleSupplier turn, DrivebaseS drivebaseS) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                drivebaseS.curvatureDrive(
                    fwdBack.getAsDouble() * 0.25,
                    turn.getAsDouble() * 0.25
                );
            },
            interrupted -> {
                drivebaseS.stopAll();
            },
            () -> false
          , drivebaseS)
          .withName("CurvatureDriveC");
    }

    public static Command createTimedDriveC(double power, double time, DrivebaseS drivebaseS) {
        return new FunctionalCommand(
            () -> {
            },
            () -> {
              drivebaseS.tankDrive(power, power);
            },
            interrupted -> {
              drivebaseS.tankDrive(0, 0);
            },
            () -> {
              return false;
            },
            drivebaseS
        )
        .withTimeout(time)
        .withName("DriveTimedC");
      }

}
