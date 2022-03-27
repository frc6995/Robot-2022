// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.IntakeS;

/** Add your docs here. */
public class IntakeCommandFactory {
    /**
     * Deploys then spins intake. When interrupted, it stops then retracts.
     * 
     * @param intakeS
     * @return run intake command
     */
    public static Command createIntakeRunC(IntakeS intakeS) {
        Command runIntake = new FunctionalCommand(
                () -> {
                    intakeS.deploy();
                },
                () -> {
                    intakeS.spin();
                },
                (interrupted) -> {
                    intakeS.stop();
                    intakeS.retract();
                },
                () -> false, intakeS);
        return runIntake;

    }

    public static Command createIntakeEjectC(IntakeS intakeS) {
        Command runIntake = new FunctionalCommand(
                () -> {
                    intakeS.deploy();
                },
                () -> {
                    intakeS.eject();
                },
                (interrupted) -> {
                    intakeS.stop();
                    intakeS.retract();
                },
                () -> false, intakeS);
        return runIntake;

    }

    /**
     * Deploys the intake
     * 
     * @param intakeS
     * @return intake deploy
     */
    public static Command createIntakeDeployC(IntakeS intakeS){
        return new InstantCommand(intakeS::deploy, intakeS);
    }

    /**
     * Spins the intake
     * 
     * @param intakeS
     * @return intake spin
     */
    public static Command createIntakeSpinC(IntakeS intakeS){
        return new RunCommand(intakeS::spin, intakeS);
    }

    /**
     * Stops the intake
     * 
     * @param intakeS
     * @return intake stop
     */
    public static Command createIntakeStopC(IntakeS intakeS){
        return new InstantCommand(intakeS::stop, intakeS);
    }

    /**
     * Retracts the intake
     * 
     * @param intakeS
     * @return intake retract
     */
    public static Command createIntakeRetractC(IntakeS intakeS){
        return new InstantCommand(intakeS::retract, intakeS);
    }

    public static Command createIntakeUntilPickupCG(IntakeS intakeS, BooleanSupplier isBallPickedUp) {
        return // Deploy intake, then
        IntakeCommandFactory.createIntakeDeployC(intakeS).andThen(
            IntakeCommandFactory.createIntakeSpinC(intakeS)
            .withInterrupt(isBallPickedUp)
        ).andThen(IntakeCommandFactory.createIntakeStopC(intakeS));
    }

    public static Command createIntakeDeployAndRunCG(IntakeS intakeS) {
        return IntakeCommandFactory.createIntakeDeployC(intakeS).andThen(
            IntakeCommandFactory.createIntakeSpinC(intakeS)
        );
    }

    public static Command createIntakeToggleRunC(IntakeS intakeS) {
        return new FunctionalCommand(
            ()->{intakeS.deploy();
            },
            ()->{intakeS.spin();},
            
            (interrupted) -> {
                intakeS.stop();
                intakeS.retract();
            },
            ()->{return false;},
            intakeS
        );
    }

    public static Command createIntakeStopAndRetractCG(IntakeS intakeS) {
        return new InstantCommand(
            () -> {
                intakeS.stop();
                intakeS.retract();
            }, intakeS);
    }
}
