package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.ShooterCommandFactory;
import frc.robot.commands.turret.TurretCommandFactory;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;

/** Primary Command Factory. */
public class MainCommandFactory {





  public static Command createVisionSpinupAndAimC(LimelightS limelightS, TurretS turretS, ShooterS shooterS) {
    return ShooterCommandFactory.createShooterDistanceSpinupC(limelightS::getFilteredDistance, shooterS)
    .alongWith(TurretCommandFactory.createTurretVisionC(limelightS, turretS));
  }
}
