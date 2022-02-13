// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.ShooterCommandFactory;
import frc.robot.commands.turret.TurretCommandFactory;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;

/** Add your docs here. */
public class MainCommandFactory {

    public static Command createAimBotC(LimelightS limelight, TurretS turretS, ShooterS shooterS) {
        return TurretCommandFactory.createTurretFollowC(limelight::getFilteredXOffset, turretS)
        .alongWith(
            ShooterCommandFactory.createShooterFollowC(
                ()->{return ShooterS.getSpeedForDistance(limelight.getFilteredDistance());},
                ()->{return ShooterS.getSpeedForDistance(limelight.getFilteredDistance());},
                shooterS)
        );
    }

}
