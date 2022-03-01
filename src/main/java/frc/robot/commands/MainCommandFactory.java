package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.MidtakeS;

/** Primary Command Factory. */
public class MainCommandFactory {

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
        IntakeCommandFactory.getIntakeDeploy(intakeS),
        // Spin until a ball is picked up, then
        new RunCommand(intakeS::spin, intakeS)
            .withInterrupt(midtakeS::getIsBottomBeamBroken)
            .andThen(intakeS::stop, intakeS),

        // Spin until the color sensor no longer detects the ball or an already-loaded
        // ball would enter the shooter
            MidtakeCommandFactory.getLoadMidtake(midtakeS)
            // Also spin the intake while the midtake is going
            .raceWith(
                IntakeCommandFactory.getIntakeSpin(intakeS)
            .andThen(MidtakeCommandFactory.getStopMidtake(midtakeS)),

        IntakeCommandFactory.getIntakeStop(intakeS),
        IntakeCommandFactory.getIntakeRetract(intakeS)
    // (the command group may be instantly restarted to intake another ball)
    // Only retract the intake if interrupted
    ))
        .withName("IntakeIndexCG");
  }
}
