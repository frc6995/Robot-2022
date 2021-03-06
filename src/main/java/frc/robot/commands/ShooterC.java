// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterS;

public class ShooterC extends CommandBase {
  /** Creates a new ShooterC. */
  private ShooterS shooter;
  private double frontTargetRPM;
  private double backTargetRPM;
  
  public ShooterC(ShooterS shooter, double frontTargetRPM, double backTargetRPM) {
    this.shooter = shooter;
    SmartDashboard.putNumber("shooterFrontSpeed", frontTargetRPM);
    SmartDashboard.putNumber("shooterBackSpeed", backTargetRPM);
    this.addRequirements(this.shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    frontTargetRPM = SmartDashboard.getNumber("shooterFrontSpeed", 0);
    backTargetRPM = SmartDashboard.getNumber("shooterBackSpeed", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this.shooter.setFrontSpeed(this.xboxController.getRightY()); 
    //this.shooter.setBackSpeed(this.xboxController.getRightY());
    shooter.pidFrontSpeed(frontTargetRPM);
    shooter.pidBackSpeed(backTargetRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.setFrontSpeed(0);
    this.shooter.setBackSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.frontAtTarget() && shooter.backAtTarget();
  }
}
