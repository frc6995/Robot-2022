// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.SuperClimberS;
import frc.robot.util.command.RunEndCommand;

/** Add your docs here. */
public class ClimberCommandFactory {
    

    public static Command createClimberBackC(SuperClimberS climberS) {
        return new RunEndCommand(climberS::tiltBackward, climberS::tiltStop, climberS.tiltClimberS);
    }

    public static Command createClimberForwardC(SuperClimberS climberS) {
        return new RunEndCommand(climberS::tiltForward, climberS::tiltStop, climberS.tiltClimberS);
    }

    public static Command createClimberExtendFrontC(SuperClimberS climberS) {
        return new RunEndCommand(climberS::extendFront, climberS::stopFront, climberS.linearClimberS);
    }

    public static Command createClimberRetractFrontC(SuperClimberS climberS) {
        return new RunEndCommand(climberS::retractFront, climberS::stopFront, climberS.linearClimberS);
    }

    public static Command createClimberRetractBackC(SuperClimberS climberS) {
        return new RunEndCommand(climberS::retractBack, climberS::stopBack, climberS.thriftyClimberS);
    }

    public static Command createClimberExtendBackC(SuperClimberS climberS) {
        return new RunEndCommand(climberS::extendBack, climberS::stopBack, climberS.thriftyClimberS);
    }

}