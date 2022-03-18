// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.climb.SuperClimberS;
import frc.robot.util.command.RunEndCommand;

/** Add your docs here. */
public class ClimberCommandFactory {
    
    //step 1
    public static Command createClimberExtendBothMaxC(SuperClimberS climberS) {
            return createClimberExtendBackC(climberS)
            .alongWith(createClimberExtendFrontC(climberS));
    }

    //step 2, raise thrifty (climberForwardC)
    // step 3 retract back

    //step 4, lower thrifty. climberBackC

    //step 5, transfer
    public static Command createClimberTransferC(SuperClimberS climberS) {
        return createClimberTransferBackC(climberS).alongWith(createClimberTransferFrontC(climberS));
    }

    //step 6, lower thrifty again




    public static Command createClimberTransferBackC(SuperClimberS climberS) {
        return new RunEndCommand(climberS::transferBack, climberS::stopBack, climberS.thriftyClimberS);
    }

    
    public static Command createClimberTransferFrontC(SuperClimberS climberS) {
        return new RunEndCommand(climberS::transferFront, climberS::stopFront, climberS.linearClimberS);
    }



    public static Command createClimberRaiseC(SuperClimberS climberS) {
        return new RunEndCommand(climberS::tiltBackward, climberS::tiltStop, climberS.tiltClimberS);
    }

    public static Command createClimberLowerC(SuperClimberS climberS) {
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