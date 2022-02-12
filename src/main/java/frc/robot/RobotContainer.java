// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShooterC;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.ShooterS;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private XboxController driverController;
  private Command xboxDriveCommand;
  private Command xboxShooterCommand;
  private DrivebaseS drivebaseS;
  private ShooterS shooterS;

  public RobotContainer() {
    // Configure the button bindings
    createControllers();
    createSubsystems();
    createCommands();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, XboxController.Button.kA.value).whileHeld(xboxShooterCommand);
  }

  private void createControllers() {
    driverController = new XboxController(Constants.USB_PORT_DRIVER_CONTROLLER);
  }

  private void createCommands() {
    xboxDriveCommand = new RunCommand(() -> {
      drivebaseS.curvatureDrive(
          driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(),
          driverController.getLeftX());
    }, drivebaseS);
    drivebaseS.setDefaultCommand(xboxDriveCommand);

    xboxShooterCommand = new ShooterC(driverController, shooterS, 0000, 5000);
    //shooterS.setDefaultCommand(xboxShooterCommand);
    
    }
  
  

  private void createSubsystems() {
    drivebaseS = new DrivebaseS();
    shooterS = new ShooterS();
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
