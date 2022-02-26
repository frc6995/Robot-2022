// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.MidtakeS;

/** Contains command methods for the midtake */
public class MidtakeCommandFactory {
    /**
     * Spins the midtake. When interrupted, the midtake stops spinning.
     * 
     * @param midtakeS
     * @return midtake run command
     */
    public static Command getMidtakeRunCommand(MidtakeS midtakeS) {
        Command runMidtake = new FunctionalCommand(
                () -> {
                },
                () -> {
                    midtakeS.spin(0.75);
                },
                (interrupted) -> {
                    midtakeS.stop();
                },
                () -> false, midtakeS);
        return runMidtake;
    }

    /**
     * Spins the mitake. Gets interrupted when the bottom beam is clear or when the top beam is broken
     * 
     * @param midtakeS
     * @return load midtake
     */
    public static Command getLoadMidtake(MidtakeS midtakeS){
        return new RunCommand(midtakeS::spin, midtakeS)
        .withInterrupt(() -> !midtakeS.getIsBottomBeamBroken())
        .withInterrupt(midtakeS::getIsTopBeamBroken);
    }

    /**
     * Stops the midtake
     * 
     * @param midtakeS
     * @return stop midtake
     */
    public static Command getStopMidtake(MidtakeS midtakeS){
        return new InstantCommand(midtakeS::stop, midtakeS);
    }
}
