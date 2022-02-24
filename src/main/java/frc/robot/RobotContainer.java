// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.MainCommandFactory;
import frc.robot.commands.auto.AutoCommandFactory;
import frc.robot.commands.drivebase.DrivebaseCommandFactory;
import frc.robot.commands.shooter.ShooterCommandFactory;
import frc.robot.commands.turret.TurretCommandFactory;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import frc.robot.util.OdometryManager;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.annotations.Log;


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
  private CommandXboxController driverController;

  @Log
  public Field2d field = new Field2d();
  // Subsystems
  private DrivebaseS drivebaseS;
  private IntakeS intakeS;
  private MidtakeS midtakeS;
  private ShooterS shooterS;
  private TurretS turretS;
  @SuppressWarnings("unused")
  private LimelightS limelightS;

  // Command
  private Command xboxDriveCommand;
  private Command runTurretC;
  private Command turretHomingC;
  private Command turretTurningC;
  private Command turretAimC;
  private Command aimbotC;
  private Command shooterSpinC;

  private OdometryManager odometryManager;

  public RobotContainer() {
    // Configure the button bindings
    createControllers();
    createSubsystems();
    odometryManager = new OdometryManager(drivebaseS::getRobotPose);
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
    driverController.b().whileActiveOnce(runTurretC);
    driverController.x().whenActive(turretHomingC);
    driverController.y().whenActive(turretTurningC);
  }

  private void createControllers() {
    driverController = new CommandXboxController(Constants.USB_PORT_DRIVER_CONTROLLER);
  }

  private void createCommands() {
    xboxDriveCommand = DrivebaseCommandFactory.createCurvatureDriveC(
      () -> {
        return driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis();
      },
      driverController::getLeftX,
      drivebaseS);
    drivebaseS.setDefaultCommand(xboxDriveCommand);

    runTurretC = TurretCommandFactory.createTurretManualC(
      driverController::getRightX, turretS);

    turretAimC = TurretCommandFactory.createTurretFollowC(odometryManager::getRotationOffset, turretS);
    turretHomingC = TurretCommandFactory.createTurretHomingC(turretS);
    turretS.setDefaultCommand(turretAimC);
    
    shooterSpinC = ShooterCommandFactory.createShooterFollowC(
          ()->{return ShooterS.getSpeedForDistance(odometryManager.getDistanceToCenter());},
          ()->{return 1.3 * ShooterS.getSpeedForDistance(odometryManager.getDistanceToCenter());},
          shooterS);
    shooterS.setDefaultCommand(shooterSpinC);
    turretTurningC = TurretCommandFactory.createTurretTurnC(40, turretS);
    SmartDashboard.putData(new InstantCommand(turretS::resetEncoder));
  }

  private void createSubsystems() {
    drivebaseS = new DrivebaseS();
    intakeS = new IntakeS();
    midtakeS = new MidtakeS();
    turretS = new TurretS();
    shooterS = new ShooterS();
    limelightS = new LimelightS();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return AutoCommandFactory.createTwoBallAutoCG(
      3800,
      1900,
      0,
      shooterS,
      intakeS,
      midtakeS,
      turretS,
      drivebaseS
    );
  }

  public void robotPeriodic() {
    field.setRobotPose(drivebaseS.getRobotPose());
    odometryManager.periodic();
    field.getObject("target").setPose(
      odometryManager.getTargetPose()
      );
    field.getObject("Turret").setPose(field.getRobotPose().transformBy(
        new Transform2d(
          new Translation2d(), Rotation2d.fromDegrees(
            turretS.getEncoderCounts()
          )
        )
      )
    );
    
  }
}
