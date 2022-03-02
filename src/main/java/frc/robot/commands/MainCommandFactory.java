package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShooterCommandFactory;
import frc.robot.commands.turret.TurretCommandFactory;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import frc.robot.util.interpolation.ShooterInterpolatingTable;

/** Primary Command Factory. */
public class MainCommandFactory {

    public static Command createIntakeUntilPickupCG(IntakeS intakeS, BooleanSupplier isBallPickedUp) {
        return // Deploy intake, then
        IntakeCommandFactory.createIntakeDeployC(intakeS).andThen(
            IntakeCommandFactory.createIntakeSpinC(intakeS)
            .withInterrupt(isBallPickedUp)
        ).andThen(IntakeCommandFactory.createIntakeStopC(intakeS));
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
        IntakeCommandFactory.createIntakeUntilPickupCG(intakeS, midtakeS::getIsBottomBeamBroken),
        MidtakeCommandFactory.createMidtakeIndexCG(midtakeS)
        // Also spin the intake while the midtake is going
        .raceWith(
            IntakeCommandFactory.createIntakeSpinC(intakeS)
        ),
        IntakeCommandFactory.createIntakeStopAndRetractCG(intakeS)
    // (the command group may be instantly restarted to intake another ball)
    )
        .withName("IntakeIndexCG");
  }

  public static Command createShooterDistanceSpinupC(DoubleSupplier distance, ShooterS shooterS) {
    return ShooterCommandFactory.createShooterFollowC(
        ()->{
          return ShooterInterpolatingTable.get(
             distance.getAsDouble()/*NomadMathUtil.getDistance(navigationManager.getTOFAdjustedRobotToHubTransform())*/
          ).frontWheelRpm;
        },
        ()->{
          return ShooterInterpolatingTable.get(
            distance.getAsDouble()/*NomadMathUtil.getDistance(navigationManager.getTOFAdjustedRobotToHubTransform())*/
          ).backWheelRpm;
        },
        shooterS);
  }

  public static Command createVisionSpinupAndAimC(LimelightS limelightS, TurretS turretS, ShooterS shooterS) {
    return createShooterDistanceSpinupC(limelightS::getFilteredDistance, shooterS)
    .alongWith(TurretCommandFactory.createTurretVisionC(limelightS, turretS));
  }
}
