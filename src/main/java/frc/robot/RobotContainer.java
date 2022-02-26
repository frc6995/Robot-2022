package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShooterTestC;
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
import io.github.oblarg.oblog.annotations.Config;
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
  private Command shooterSpinC;
  @Log
  @Config
  private Command shooterTestC;

  private Trigger targetDistanceInRangeTrigger;
  private Trigger shooterReadyTrigger;
  private Trigger turretReadyTrigger;
  private Trigger ballReadyTrigger;

  private Trigger shootBallTrigger;

  private OdometryManager odometryManager;

  public RobotContainer() {
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
    Command runIntake = new FunctionalCommand(
        () -> {
          intakeS.deploy();
        },
        () -> {
          intakeS.spin();
        },
        (interrupted) -> {
          intakeS.stop();
          intakeS.retract();
        },
        () -> false, intakeS);

    Command runMidtake = new FunctionalCommand(
        () -> {
        },
        () -> {
          midtakeS.spin(0.75);
        },
        (interrupted) -> {
          midtakeS.stop();
        },
        () -> false, midtakeS);

    targetDistanceInRangeTrigger = new Trigger(odometryManager::getDistanceInRange);
    turretReadyTrigger = new Trigger(turretS::isAtTarget);
    shooterReadyTrigger = new Trigger(shooterS::isAtTarget);
    ballReadyTrigger = new Trigger(midtakeS::getIsTopBeamBroken); // TODO fix this logic

    //targetDistanceInRangeTrigger.whileActiveContinuous(shooterSpinC);

    shootBallTrigger = targetDistanceInRangeTrigger.and(turretReadyTrigger).and(shooterReadyTrigger).and(ballReadyTrigger);

    shootBallTrigger.whenActive(runMidtake);
    driverController.b().whileActiveOnce(runMidtake);
    driverController.x().whileActiveOnce(shooterTestC);
    driverController.y().whenActive(turretTurningC);
    driverController.a().whileActiveOnce(runIntake);
  }

  /**
   * Instantiate the driver and operator controllers
   */
  private void createControllers() {
    driverController = new CommandXboxController(Constants.USB_PORT_DRIVER_CONTROLLER);
  }

  /**
   * Instantiate the commands
   */
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
          ()->{return ShooterS.getSpeedForDistance(odometryManager.getDistanceToCenter(), false);},
          ()->{return ShooterS.getSpeedForDistance(odometryManager.getDistanceToCenter(), true);},
          shooterS);
    turretTurningC = TurretCommandFactory.createTurretTurnC(40, turretS);
    shooterTestC = new ShooterTestC(shooterS);
    //shooterS.setDefaultCommand(ShooterCommandFactory.createShooterIdleC(shooterS));
    SmartDashboard.putData(new InstantCommand(turretS::resetEncoder));
  }

  /**
   * Instantiate the subsystems
   */
  private void createSubsystems() {
    drivebaseS = new DrivebaseS();

    intakeS = new IntakeS();
    midtakeS = new MidtakeS();
    turretS = new TurretS();
    shooterS = new ShooterS();
    odometryManager = new OdometryManager(drivebaseS::getRobotPose, turretS::getRotation2d, turretS::setTransformVelocity);
    limelightS = new LimelightS(odometryManager,
    (List<Pose2d> list)->{field.getObject("targetRing").setPoses(list);});
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
        drivebaseS);
  }

  public void robotPeriodic() {


    /*Field2d setup */
    field.setRobotPose(drivebaseS.getRobotPose());
    odometryManager.periodic();

    field.getObject("target").setPose(new Pose2d(
      odometryManager.getCurrentRobotPose().getTranslation().plus(
        odometryManager.getRobotToHub().getTranslation().rotateBy(
          odometryManager.getCurrentRobotPose().getRotation())),
    Rotation2d.fromDegrees(0)));

//     field.getObject("cameraTarget").setPose(
//       odometryManager.getCurrentRobotPose().transformBy(
//         odometryManager.getRobotToCamera()).transformBy(
//           odometryManager.getCameraToHub()));
    field.getObject("Turret").setPose(odometryManager.getCurrentRobotPose().transformBy(
        odometryManager.getRobotToTurret()
      )
    );

    field.getObject("camera").setPose(odometryManager.getCurrentRobotPose().transformBy(odometryManager.getRobotToCamera()));
    
  }
}
