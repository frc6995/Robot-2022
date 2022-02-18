// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivebaseS;

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
  private DrivebaseS drivebaseS;

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
  }

  private void createSubsystems() {
    drivebaseS = new DrivebaseS();
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

  public Command createTimedDriveC(double power, double time, DrivebaseS drivebaseS) {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          drivebaseS.tankDrive(power, power);
        },
        (Boolean interrupted) -> {
          drivebaseS.tankDrive(0, 0);
        },
        () -> {
          return false;
        },
        drivebaseS)
            .withTimeout(time);
  }



  /**
   * Creates a command group to complete a two ball auto: spins up the shooter, sets the front and 
   * back shooter motors, homes the turret, drives, intakes one ball, retracts the intake, and spins midtake to
   * shoot one ball.
   * 
   * @param targetFrontSpeed the target speed of the front shooter motor
   * @param targetBackSpeed the target speed of the back shooter motor
   * @param targetAngle the target angle of the turret
   * @param shooterS the shooter subsystem
   * @param intakeS the intake subsystem
   * @param midtakeS the midtake subsystem
   * @param turretS the turret subsystem
   * @param drivebaseS the drivebase subsystem
   * @return the two ball auto command group
   */
  public Command createTwoBallAutoCG(double targetFrontSpeed, double targetBackSpeed, double targetAngle,
      ShooterS shooterS, IntakeS intakeS, MidtakeS midtakeS, TurretS turretS, DrivebaseS drivebaseS) {
    return new ParallelCommandGroup(
        // Spins up the front and back shooter motors to the set target speeds
        new InstantCommand(() -> {
          shooterS.pidFrontSpeed(targetFrontSpeed);
          shooterS.pidBackSpeed(targetBackSpeed);
        },
            shooterS)
                .andThen(
                    new WaitUntilCommand(shooterS::isAtSetpoint)),
        // Homes the turret
        createTurretHomeC(turretS)
            .andThen(
                () -> {
                  turretS.runPid(targetAngle);
                }, turretS)
            .andThen(
                new WaitUntilCommand(turretS::isAtSetpoint)),
        // Drives at 25% speed for 7 seconds or until the color sensor detects a ball present
        createTimedDriveC(.25, 7, drivebaseS)
            .withInterrupt(midtakeS::getBallPresent),
        // Intakes one ball, stores or rejects based on whether the color is correct
        createIntakeIndexCG(intakeS, midtakeS)
            // Retracts intake
            .andThen(intakeS::retract, intakeS))
                // Spins the midtake to feed the shooter
                .andThen(midtakeS::spin, midtakeS).withName("Two Ball Auto");
  }
}
