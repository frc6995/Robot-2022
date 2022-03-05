// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.MidtakeS;

/** Contains command methods for the midtake */
public class MidtakeCommandFactory {
    /**
     * Spins the midtake. When interrupted, the midtake stops spinning.
     * 
     * @param midtakeS
     * @return midtake run command
     */
    public static Command createMidtakeRunC(MidtakeS midtakeS) {
        Command runMidtake = new FunctionalCommand(
                () -> {
                },
                midtakeS::load,
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
    public static Command createMidtakeLoadC(MidtakeS midtakeS){
        return new RunCommand(midtakeS::load, midtakeS);
    }

    public static Command createMidtakeIndexCG(MidtakeS midtakeS) {
        return new RepeatCommand(
            new WaitUntilCommand(midtakeS::getIsBottomBeamBroken).alongWith(
                new WaitUntilCommand(()->!midtakeS.getIsTopBeamBroken())
            )
            
            .andThen(
                createMidtakeLoadC(midtakeS)
                .withInterrupt(()->!midtakeS.getIsBottomBeamBroken())
                .withInterrupt(midtakeS::getIsTopBeamBroken)
            )
            .andThen(
                createMidtakeStopC(midtakeS)
            )
        );
    }

    public static Command createMidtakeFeedOneC(MidtakeS midtakeS) {
        return createMidtakeFeedC(midtakeS).withInterrupt(midtakeS.cargoLeftTrigger);
    }

    /**
     * Stops the midtake
     * 
     * @param midtakeS
     * @return stop midtake
     */
    public static Command createMidtakeStopC(MidtakeS midtakeS){
        return new InstantCommand(midtakeS::stop, midtakeS);
    }

    public static Command createMidtakeFeedC(MidtakeS midtakeS) {
        return new RunCommand(midtakeS::feed, midtakeS);
    }
}
