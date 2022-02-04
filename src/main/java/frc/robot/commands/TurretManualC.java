// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretS;
/**
 * control the turret
 */
public class TurretManualC extends CommandBase {
  private XboxController turretController;
  private TurretS turretS;

  /** Creates a new TurretManualC. */
  public TurretManualC(XboxController xboxController, TurretS turretS) {
    this.turretController = xboxController;
    this.turretS = turretS;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.turretS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /**
   * sets the turn speed of the turret to the horizontal movement of the right joystick
   */
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretS.turnSpeed(turretController.getRightX());
  }
/**
 * stops the turret
 */
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretS.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
