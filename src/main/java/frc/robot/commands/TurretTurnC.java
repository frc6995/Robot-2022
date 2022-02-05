// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretS;

public class TurretTurnC extends CommandBase {
  private TurretS turret;
  /** Creates a new TurretTurnC. */
  public TurretTurnC(TurretS turret) {
    this.turret = turret;
    addRequirements(this.turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double difference = 40 - turret.getEncoderCounts(); 
    SmartDashboard.putNumber("turret error", difference);
    turret.turnSpeed((difference * 0.0025) + (0.02 * Math.signum(difference)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean stuff = 39 < turret.getEncoderCounts();
    boolean otherStuff = turret.getEncoderCounts() < 41;
    return stuff && otherStuff;
  }
}
