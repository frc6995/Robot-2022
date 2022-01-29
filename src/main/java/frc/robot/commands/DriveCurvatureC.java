// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseS;

public class DriveCurvatureC extends CommandBase {
  private DrivebaseS drivebase;
  private XboxController xboxController;
  /** Creates a new DriveCurvatureC. */
  public DriveCurvatureC(DrivebaseS drivebaseS, XboxController xboxController) {
    this.drivebase = drivebaseS;
    this.xboxController = xboxController;
    this.addRequirements(this.drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Nothing needs to be done

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.drivebase.curvatureDrive(xboxController.getRightTriggerAxis() - xboxController.getLeftTriggerAxis(), 
    xboxController.getLeftX());
  }
  // Curvature Drive using the Triggers and the Left Joystick

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivebase.curvatureDrive(0, 0);
  }
  // Stop the motors

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
