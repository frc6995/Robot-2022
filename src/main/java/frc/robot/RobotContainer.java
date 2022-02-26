// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberBackC;
import frc.robot.commands.ClimberExtendsC;
import frc.robot.commands.ClimberForwardC;
import frc.robot.commands.ClimberRetractsC;
import frc.robot.commands.ClimberForwardC;
import frc.robot.commands.ClimberBackC;
import frc.robot.commands.ShooterC;
import frc.robot.subsystems.ClimberS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import frc.robot.commands.turret.TurretCommandFactory;


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
  private XboxController operatorController;
  private Command xboxDriveCommand;
  private Command xboxShooterCommand;
  private DrivebaseS drivebaseS;
  private ShooterS shooterS;
  private TurretS turretS;
  private ClimberS climberS;

  
  private Command runTurretC;
  private Command turretHomingC;
  private Command turretTurningC;
  private Command climberForwardC;
  private Command climberBackC;

  // Trigger definitions
  private Trigger spinTurretTrigger;
  private Trigger turretHomeTrigger;
  private Trigger turretTurnTrigger;
  //private Trigger extendsClimber;
  //private Trigger retractClimber;
  private Trigger climberForward;
  private Trigger climberBack;

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

    //new Trigger(driverController::getAButton).whileActiveOnce(xboxShooterCommand);
    /*spinTurretTrigger = new Trigger(driverController::getBButton);
    spinTurretTrigger.whileActiveOnce(runTurretC);
    turretHomeTrigger = new Trigger(driverController::getXButton);
    turretHomeTrigger.whenActive(turretHomingC);
    turretTurnTrigger = new Trigger(driverController::getYButton);
    turretTurnTrigger.whenActive(turretTurningC);*/
    //extendsClimber = new Trigger(operatorController::getAButton);
    //extendsClimber.whenActive(new ClimberExtendsC(climberS));
    //retractClimber = new Trigger(operatorController::getBButton);
    //retractClimber.whenActive(new ClimberRetractsC(climberS));
    climberForward = new Trigger(operatorController::getAButton);
    climberForward.whileActiveOnce(climberForwardC);
    climberBack = new Trigger(operatorController::getBButton);
    climberBack.whileActiveOnce(climberBackC);

  }

  private void createControllers() {
    driverController = new XboxController(Constants.USB_PORT_DRIVER_CONTROLLER);
    operatorController = new XboxController(Constants.USB_PORT_OPERATOR_CONTROLLER);
  }

  private void createCommands() {
    xboxDriveCommand = new RunCommand(
      ()
     -> {drivebaseS.curvatureDrive(
       driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(), 
       driverController.getLeftX()
       );
      }
    , drivebaseS);
    drivebaseS.setDefaultCommand(xboxDriveCommand);

    runTurretC = TurretCommandFactory.createTurretManualC(
      driverController::getRightX, turretS);

    turretS.setDefaultCommand(runTurretC);
    
    turretHomingC = TurretCommandFactory.createTurretHomingC(turretS);

    turretTurningC = TurretCommandFactory.createTurretTurnC(40, turretS);
    
    SmartDashboard.putData(new InstantCommand(turretS::resetEncoder));

    climberForwardC = new ClimberForwardC(climberS);
    climberBackC = new ClimberBackC(climberS);
  }

  private void createSubsystems() {
    drivebaseS = new DrivebaseS();
    turretS = new TurretS();
    climberS = new ClimberS();
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
