// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberS;

/**
 * Extends the climber a certain amount of rotations
 */
public class ClimberExtendsC extends CommandBase {
  /** Creates a new ClimberC. */
  private ClimberS climber;
  public ClimberExtendsC(ClimberS climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setClimberRotations(9);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.atTarget();
  }
}
