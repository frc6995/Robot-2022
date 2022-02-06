// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeS;

public class IntakeC extends CommandBase {
   /** Creates a new IntakeC. */
   private IntakeS intake;
  /** Makes a new IntakeC */
  public IntakeC(IntakeS intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * Deploys the intake and starts the intake motor speed
   */
  @Override
  public void initialize() {
    intake.intakeDeploy();
    intake.intakeMotor(Constants.INTAKE_MOTOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  /**
   * Retracts the intake and stops the intake motor
   */
  @Override
  public void end(boolean interrupted) {
    intake.intakeRetract();
    intake.intakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
