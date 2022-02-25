package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.shooter.ShooterCommandFactory;
import frc.robot.commands.turret.TurretCommandFactory;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import frc.robot.util.OdometryManager;

/** Primary Command Factory. */
public class MainCommandFactory {

  /**
   * Create a command to aim at the target with the turret and shooter
   * 
   * @param odometryManager The OdometryManager 
   * @param turretS   The turret subsystem
   * @param shooterS  The shooter subsystem
   * @return The aimbot command
   */
    public static Command createAimBotC(OdometryManager odometryManager, TurretS turretS, ShooterS shooterS) {
        return TurretCommandFactory.createTurretFollowC(odometryManager::getRotationOffset, turretS)
        .alongWith(
            ShooterCommandFactory.createShooterFollowC(
                ()->{
                  return ShooterS.getSpeedForDistance(odometryManager.getDistanceToCenter(), false);
                },
                ()->{
                  return ShooterS.getSpeedForDistance(odometryManager.getDistanceToCenter(), true);
                },
                shooterS)
        );
    }

  /**
   * Creates a command group to intake and store or reject one ball.
   * 
   * @param intakeS the intake subsystem
   * @param midtakeS the midtake subsystem
   * @return the "intake and index" command group
   */

  public static Command createIntakeIndexCG(IntakeS intakeS, MidtakeS midtakeS) {
    return new SequentialCommandGroup(
        // Deploy intake, then
        new InstantCommand(intakeS::deploy, intakeS),
        // Spin until a ball is picked up, then
        new RunCommand(intakeS::spin, intakeS)
            .withInterrupt(midtakeS::getColorSensorDetectsBall)
            .andThen(intakeS::stop, intakeS),
        // Decide whether to store or reject, and do so
        new ConditionalCommand(
            // If the color is correct
            // Spin until the color sensor no longer detects the ball or an already-loaded
            // ball would enter the shooter
            new RunCommand(midtakeS::spin, midtakeS)
                .withInterrupt(() -> !midtakeS.getColorSensorDetectsBall())
                .withInterrupt(midtakeS::getIsTopBeamBroken)
                // Also spin the intake while the midtake is going
                .raceWith(
                    new RunCommand(intakeS::spin, intakeS))
                .andThen(midtakeS::stop, midtakeS),
            // If the color is wrong
            // Eject the ball for 3 seconds
            new RunCommand(intakeS::eject, intakeS)
                .withTimeout(3),
            // choose between the above two commands based on the color sensor
            midtakeS::getIsBallColorCorrect),
        new InstantCommand(intakeS::stop, intakeS)
    // Don't retract the intake
    // (the command group may be instantly restarted to intake another ball)
    // Only retract the intake if interrupted
    )
      .withName("IntakeIndexCG");
  }

}
