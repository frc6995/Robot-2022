// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.MidtakeS;
import frc.robot.util.command.RunEndCommand;

/** Contains command methods for the midtake */
public class MidtakeCommandFactory {

    public static Command createMidtakeReverseC(MidtakeS midtakeS) {
        return new RunEndCommand(midtakeS::reverse, midtakeS::stop, midtakeS);
    }

    /**
     * Spins the mitake. Gets interrupted when the bottom beam is clear or when the top beam is broken
     * 
     * @param midtakeS
     * @return load midtake
     */
    public static Command createMidtakeLoadC(MidtakeS midtakeS){
        return new RunEndCommand(midtakeS::load, midtakeS::stop, midtakeS);
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
        return new RunEndCommand(midtakeS::feed, midtakeS::stop, midtakeS);
    }

    
    public static Command createMidtakeCrawlC(MidtakeS midtakeS) {
        return new RunEndCommand(midtakeS::crawl, midtakeS::stop, midtakeS);
    }

    // END BASIC OPERATION

    /**
     * Waits until the bottom beam is broken and the top is clear, then loads until the bottom is clear or the top is broken.
     * 
     * Has the effect of processing balls as the intake brings them in, and stopping the balls as low as possible.
     * @param midtakeS
     * @return
     */

    public static Command createMidtakeLowIndexCG(MidtakeS midtakeS) {
        return new RepeatCommand(
            new WaitUntilCommand(midtakeS::getIsBottomBeamBroken).alongWith(
                new WaitUntilCommand(()->!midtakeS.getIsTopBeamBroken())
            )
            .andThen(
                createMidtakeLoadC(midtakeS)
                .withInterrupt(()->!midtakeS.getIsBottomBeamBroken())
                .withInterrupt(midtakeS::getIsTopBeamBroken)
            )
        );
    }

    /**
     * If a ball is breaking the top beam, drives the ball down until it breaks the bottom beam.
     * @param midtakeS
     * @return
     */
    public static Command createMidtakeReadyIntakeCG(MidtakeS midtakeS) {
        return new ConditionalCommand(
            createMidtakeReverseC(midtakeS)
            .withInterrupt(midtakeS::getIsBottomBeamBroken),
            new InstantCommand(),
            midtakeS::getIsTopBeamBroken
        );
    }

    /**
     * Backs up the midtake to recover a ball that is forward of the top beam break.
     * 
     * To be run after a shot. The MidtakeArmCG will bring the ball back to the firing position before a shot.
     * @param midtakeS
     * @return
     */
    public static Command createMidtakePostShotC(MidtakeS midtakeS) {
        return new ConditionalCommand(
            createMidtakeReverseC(midtakeS)
            .withInterrupt(
                ()->!midtakeS.getIsTopBeamBroken()
            ).withTimeout(1.0),
            new InstantCommand(),
            midtakeS::getIsTopBeamBroken
        );
    }




    public static Command createMidtakeArmC(MidtakeS midtakeS) {
        /*
        To be run when preparing to fire a ball. 
        It ensures the next ball to fire is stopped below the top beam break, but just barely breaking it.
        If no ball is actually in the intake, it will try to arm for roughly a second before timing
        */
        return new RepeatCommand(
            createMidtakeLoadC(midtakeS)
            .withInterrupt(midtakeS::getIsTopBeamBroken)
            .withTimeout(1.0)
        );
    }

    /**
     * Runs a short burst at high speed to feed one ball into the shooter. 
     * @param midtakeS
     * @return
     */
    public static Command createMidtakeFeedOneC(MidtakeS midtakeS) {

        return createMidtakeFeedC(midtakeS)
            .withInterrupt(()->!midtakeS.getIsTopBeamBroken())
            .withTimeout(0.15)
        .andThen(
            createMidtakeStopC(midtakeS)
        );
    }
    /**
     * By default, the midtake will perpetually drive slowly towards the top except when the top beam is broken,
     * to ensure that if a ball is in the midtake, it is situated up against the top beam, ready to fire.
     * @param midtakeS
     * @return
     */
    public static Command createMidtakeDefaultC(MidtakeS midtakeS) {
        return new ConditionalCommand(
            new InstantCommand(),
            createMidtakeCrawlC(midtakeS).withInterrupt(midtakeS::getIsTopBeamBroken),
            midtakeS::getIsTopBeamBroken
        );
    }
}
