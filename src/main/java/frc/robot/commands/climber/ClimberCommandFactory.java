// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberS;
import frc.robot.util.command.RunEndCommand;

/** Add your docs here. */
public class ClimberCommandFactory {
    public static Command createClimberBackC(ClimberS climberS) {
        return new RunEndCommand(climberS::tiltBackward, climberS::tiltStop, climberS);
    }

    public static Command createClimberForwardC(ClimberS climberS) {
        return new RunEndCommand(climberS::tiltForward, climberS::tiltStop, climberS);
    }

    public static Command createClimberExtendFrontC(ClimberS climberS) {
        return new RunEndCommand(climberS::extendFront, climberS::stopFront, climberS);
    }

    public static Command createClimberRetractFrontC(ClimberS climberS) {
        return new RunEndCommand(climberS::retractFront, climberS::stopFront, climberS);
    }

    public static Command createClimberRetractBackC(ClimberS climberS) {
        return new RunEndCommand(climberS::retractBack, climberS::stopBack, climberS);
    }

    public static Command createClimberExtendBackC(ClimberS climberS) {
        return new RunEndCommand(climberS::extendBack, climberS::stopBack, climberS);
    }

}