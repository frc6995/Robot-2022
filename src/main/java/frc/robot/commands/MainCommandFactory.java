package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.shooter.ShooterCommandFactory;
import frc.robot.commands.turret.TurretCommandFactory;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LightS;
import frc.robot.subsystems.LightS.States;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.pose.NavigationManager;

/** Primary Command Factory. */
public class MainCommandFactory {

  // Intake deploy -> midtake prepare - > intake spin...on end, midtake arm, intake stop and retract.

  public static Command createIntakeCG(MidtakeS midtakeS, IntakeS intakeS) {
    return new ConditionalCommand(
      new InstantCommand(), // if full, do nothing
      new ConditionalCommand(
        new ParallelCommandGroup(
          IntakeCommandFactory.createIntakeDeployC(intakeS),
          MidtakeCommandFactory.createMidtakeReadyIntakeCG(midtakeS)
        ),
        IntakeCommandFactory.createIntakeDeployC(intakeS),
        ()->{return midtakeS.getBallCount() == 1;}
      ).andThen(
        new ParallelCommandGroup(
          IntakeCommandFactory.createIntakeRunC(intakeS),
          MidtakeCommandFactory.createMidtakeLowIndexCG(midtakeS),
          new RunCommand(()->{LightS.getInstance().requestState(States.Intaking);})
        )
      ),
      midtakeS::getIsMidtakeFull
    );
  }
  


  public static Command createVisionSpinupAndAimC(LimelightS limelightS, TurretS turretS, ShooterS shooterS) {
    return ShooterCommandFactory.createShooterDistanceSpinupC(limelightS::getFilteredDistance, shooterS)
    .alongWith(TurretCommandFactory.createTurretVisionC(limelightS, turretS));
  }

  public static Command createShooterOdometryC(NavigationManager navigationManager, ShooterS shooterS) {
    return ShooterCommandFactory.createShooterDistanceSpinupC(()->NomadMathUtil.getDistance(navigationManager.getRobotToHubTransform()), shooterS);
  }

  public static Command createWrongBallC(NavigationManager navigationManager, TurretS turretS, ShooterS shooterS) {
    return new ConditionalCommand(
      TurretCommandFactory.createTurretWrongBallC(
      navigationManager::getRobotToHubDirection,
      navigationManager::getRobotToHubDistance, turretS)
      .alongWith(createShooterOdometryC(navigationManager, shooterS)),
      ShooterCommandFactory.createShooterIdleC(shooterS)
      .alongWith(createTurretOdometryC(navigationManager, turretS)),
      ()->{
        return turretS.isTargetInRange(
          navigationManager.getRobotToHubDirection()
        );
      });
  }

  public static Command createTurretOdometryC(NavigationManager navigationManager, TurretS turretS) {
    return TurretCommandFactory.createTurretFollowC(
      ()->{
        Rotation2d direction = NomadMathUtil.getDirection(navigationManager.getRobotToHubTransform());
        return direction;
      }, turretS);
  }

  /**
   * Shooter default command: Spin up for proper distance.
   * Turret default command: Aim at hub (within limits)
   * Midtake default command: idle
  */
}
