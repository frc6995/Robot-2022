// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberS;

/**
 * Retracts the back arms 
 */
public class ClimberRetractsArmTwoC extends CommandBase {
  /** Creates a new ClimberRetractsArmTwoC. */
  ClimberS climber;
  public ClimberRetractsArmTwoC(ClimberS climbers) {
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
    climber.setClimberRotations(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotorArmTwo();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.atTarget();
  }
}
