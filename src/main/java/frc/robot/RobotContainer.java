// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretManualC;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.TurretS;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private XboxController turretController;
  private Command xboxDriveCommand;
  private DrivebaseS drivebaseS;
  private TurretS turretS;
  private Command runTurretC;


  // Trigger definitions
  private Trigger spinTurretTrigger;

  public RobotContainer() {
    // Configure the button bindings
    createControllers();
    createSubsystems();
    createCommands();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    spinTurretTrigger = new Trigger(turretController::getAButton);
    spinTurretTrigger.whileActiveOnce(runTurretC);
  }

  private void createControllers() {
    turretController = new XboxController(Constants.USB_PORT_DRIVER_CONTROLLER);
  }

  private void createCommands() {
    xboxDriveCommand = new RunCommand(()
     -> {drivebaseS.curvatureDrive(
       turretController.getRightTriggerAxis() - turretController.getLeftTriggerAxis(), 
       turretController.getLeftX()
       );
      }
    , drivebaseS);
    drivebaseS.setDefaultCommand(xboxDriveCommand);
    runTurretC = new TurretManualC(turretController, turretS);
    turretS.setDefaultCommand(runTurretC);
    


    SmartDashboard.putData(new InstantCommand(turretS::resetEncoder));
  }

  private void createSubsystems() {
    drivebaseS = new DrivebaseS();
    turretS = new TurretS();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new WaitCommand(0);
  }
}
