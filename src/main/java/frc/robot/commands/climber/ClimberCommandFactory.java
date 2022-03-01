// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberS;

/** Add your docs here. */
public class ClimberCommandFactory {
    public static Command createClimberBackC(ClimberS climber) {
        return new FunctionalCommand(
            climber::climberDown,
            ()->{},
            (interrupted)->{climber.climberOff();},
            ()->false,
            climber);
    }

    public static Command createClimberExtendArmTwoC(ClimberS climber) {
        return new FunctionalCommand(
            ()->{},
            ()->{climber.setClimberRotations(9);},
            (interrupted)->{climber.stopMotorArmTwo();},
            climber::atTarget,
            climber);
    }

    public static Command createClimberExtendC(ClimberS climber) {
        return new FunctionalCommand(
            ()->{},
            ()->{climber.setClimberRotations(9);},
            (interrupted)->{climber.stopMotor();},
            climber::atTarget,
            climber);
    }

    public static Command createClimberFoldDownC(ClimberS climber) {
        return new InstantCommand(climber::climberDown);
    }

    public static Command createClimberFoldUpC(ClimberS climber) {
        return new InstantCommand(climber::climberUp);
    }

    public static Command createClimberForwardC(ClimberS climber) {
        return new FunctionalCommand(climber::climberUp,
        ()->{},
        (interrupted) -> {climber.climberOff();}, ()->false, climber);
    }

    public static Command createClimberRetractArmTwoC(ClimberS climber) {
        return new FunctionalCommand(()->{}, ()->{climber.setClimberRotations(0);}, (interrupted) -> {climber.stopMotorArmTwo();}, climber::atTarget, climber);
    }

    public static Command createClimberRetractC(ClimberS climber) {
        return new FunctionalCommand(()->{}, ()->{climber.setClimberRotations(0);}, (interrupted) -> {climber.stopMotor();}, climber::atTarget, climber);
    }
}