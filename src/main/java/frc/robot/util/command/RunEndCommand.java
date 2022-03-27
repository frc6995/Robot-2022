// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.command;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that allows the user to pass in functions for each of the basic command methods through
 * the constructor. Useful for inline definitions of complex commands - note, however, that if a
 * command is beyond a certain complexity it is usually better practice to write a proper class for
 * it than to inline it.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class RunEndCommand extends CommandBase {
  protected final Runnable m_onExecute;
  protected final Runnable m_onEnd;

  /**
   * Creates a new FunctionalCommand.
   *
   * @param onInit the function to run on command initialization
   * @param onExecute the function to run on command execution
   * @param onEnd the function to run on command end
   * @param isFinished the function that determines whether the command has finished
   * @param requirements the subsystems required by this command
   */
  public RunEndCommand(
      Runnable onExecute,
      Runnable onEnd,
      Subsystem... requirements) {
    m_onExecute = requireNonNullParam(onExecute, "onExecute", "FunctionalCommand");
    m_onEnd = requireNonNullParam(onEnd, "onEnd", "FunctionalCommand");

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_onExecute.run();
  }

  @Override
  public void end(boolean interrupted) {
    m_onEnd.run();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
