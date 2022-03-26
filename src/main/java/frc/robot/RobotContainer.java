package frc.robot;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommandFactory;
import frc.robot.commands.MainCommandFactory;
import frc.robot.commands.MidtakeCommandFactory;
import frc.robot.commands.ShooterTestC;
import frc.robot.commands.auto.AutoCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.drivebase.DrivebaseCommandFactory;
import frc.robot.commands.shooter.ShooterCommandFactory;
import frc.robot.commands.turret.TurretCommandFactory;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LightS;
import frc.robot.subsystems.LightS.States;
import frc.robot.subsystems.climb.SuperClimberS;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.command.RunEndCommand;
import frc.robot.util.pose.NavigationManager;
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
  private CommandXboxController operatorController;

  @Log
  public Field2d field = new Field2d();
  // Subsystems
  private DrivebaseS drivebaseS;
  private IntakeS intakeS;
  private MidtakeS midtakeS;
  private ShooterS shooterS;
  private TurretS turretS;
  private SuperClimberS climberS;
  private LimelightS limelightS;

  // Command
  private Command xboxDriveCommand;
  private Command runTurretC;
  private Command turretManualC;
  private Command turretAimC;
  private Command turretOdometryC;
  private Command shooterVisionSpinC;
  private Command shooterOdometryC;
  private Command shooterTarmacLineC;
  private Command visionSpinAndAimC;
  private Command odometrySpinAndAimC;
  private Command midtakeFeedC;
  private Command midtakeFeedOneC;
  private Command midtakeDefaultC;
  private Command midtakeManualC;
  private Command runIntakeC;
  private Command manualIntakeCG;
  private Command climberRetractTwoC;
  private Command climberExtendTwoC;
  @Log
  private Command turretFrictionTestC;
  @Log
  @Config
  private Command shooterTestC;

  private NetworkTable spinAndAimChooserTable = NetworkTableInstance.getDefault().getTable("Shuffleboard")
      .getSubTable("RobotContainer/Spinup Method");
  private SendableChooser<Character> spinAndAimChooser = new SendableChooser<>();

  @Log
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private NavigationManager navigationManager;

  @Log(methodName = "get")
  private Trigger shooterReadyTrigger;
  @Log(methodName = "get")
  private Trigger turretReadyTrigger;
  @Log(methodName = "get")
  private Trigger ballReadyTrigger;
  @Log(methodName = "get")
  private Trigger drivebaseStoppedTrigger;
  @Log(methodName = "get")
  private Trigger shootButtonTrigger;
  @Log(methodName = "get")
  private Trigger distanceInRangeTrigger;

  @Log(methodName = "get")
  private Trigger shootBallTrigger;
  @Log(methodName = "get")
  private Trigger dumpWrongBallTrigger;

  private Trigger spinAndAimTrigger;

  public RobotContainer() {
    createControllers();
    createSubsystems();

    createCommands();
    createTriggers();
    configureButtonBindings();
    navigationManager.setVisionEnabledSupplier(() -> true);
    CameraServer.startAutomaticCapture();
  }

  /**
   * Instantiate the driver and operator controllers
   */
  private void createControllers() {
    driverController = new CommandXboxController(Constants.USB_PORT_DRIVER_CONTROLLER);
    operatorController = new CommandXboxController(Constants.USB_PORT_OPERATOR_CONTROLLER);
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
    climberS = new SuperClimberS();
    navigationManager = new NavigationManager(
        drivebaseS::getRobotPose,
        turretS::getRobotToTurretRotation,
        turretS::setTransformVelocity);
    limelightS = new LimelightS(
        navigationManager,
        (list) -> {
          field.getObject("targetRing").setPoses(list);
        });
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

    turretAimC = TurretCommandFactory.createTurretVisionC(limelightS, turretS);
    turretManualC = TurretCommandFactory.createTurretManualC(() -> -operatorController.getLeftX(), turretS);
    // turretS.setDefaultCommand(turretDefaultC);
    turretS.setDefaultCommand(turretManualC);

    shooterVisionSpinC = ShooterCommandFactory.createShooterDistanceSpinupC(limelightS::getFilteredDistance, shooterS);
    shooterTestC = new ShooterTestC(shooterS);
    shooterTarmacLineC = ShooterCommandFactory.createShooterFollowC(() -> 500, () -> 500, shooterS);

    visionSpinAndAimC = MainCommandFactory.createVisionSpinupAndAimC(limelightS, turretS, shooterS);
    odometrySpinAndAimC = new ConditionalCommand(
        MainCommandFactory.createVisionSpinupAndAimC(limelightS, turretS, shooterS),
        MainCommandFactory.createTurretOdometryC(navigationManager, turretS)
            .alongWith(
                MainCommandFactory.createShooterOdometryC(
                    navigationManager, shooterS)),
        limelightS.hasSteadyTarget);

    autoChooser.addOption("taxi", DrivebaseCommandFactory.createTimedDriveC(0.3, 3, drivebaseS));
    autoChooser.addOption("4 Ball Auto",
        AutoCommandFactory.createFourBallAuto(shooterS, intakeS, midtakeS, turretS, limelightS, drivebaseS));
    autoChooser.setDefaultOption("2 ball",
        AutoCommandFactory.createTwoBallAutoCG(shooterS, intakeS, midtakeS, turretS, limelightS, drivebaseS));
    // autoChooser.setDefaultOption("3 ball",
    // AutoCommandFactory.createThreeBallAutoCG(shooterS, intakeS, midtakeS,
    // turretS, limelightS, drivebaseS));

    runIntakeC = MainCommandFactory.createIntakeCG(midtakeS, intakeS);
    manualIntakeCG = IntakeCommandFactory.createIntakeRunC(intakeS);

    midtakeDefaultC = MidtakeCommandFactory.createMidtakeDefaultC(midtakeS);
    midtakeFeedC = MidtakeCommandFactory.createMidtakeFeedC(midtakeS);
    midtakeFeedOneC = MidtakeCommandFactory.createMidtakeFeedOneC(midtakeS);
    midtakeManualC = MidtakeCommandFactory.createMidtakeManualC(operatorController::getRightY, midtakeS);
    midtakeS.setDefaultCommand(
        midtakeDefaultC);
  }

  public void createTriggers() {
    shooterReadyTrigger = new Trigger(shooterS::isAtTarget)
        .and(new Trigger(() -> (shooterS.getFrontEncoderSpeed() > 100)));
    shootButtonTrigger = new Trigger(
        () -> {
          return (operatorController.getRightTriggerAxis() >= 0.5);
        });
    distanceInRangeTrigger = new Trigger(() -> {
      return NavigationManager.getDistanceInRange(limelightS.getFilteredDistance());
    });

    spinAndAimTrigger = new Trigger(() -> {
      return (operatorController.getLeftTriggerAxis() >= 0.5);
    });

    shootBallTrigger = shootButtonTrigger
        .and(shooterReadyTrigger.debounce(0.2));

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
    driverController.a().toggleWhenActive(
        MainCommandFactory.createIntakeCG(midtakeS, intakeS)
            .alongWith(
                new RunCommand(
                    () -> {
                      LightS.getInstance().requestState(States.Intaking);
                    })));

    driverController.y().whileActiveContinuous(
        IntakeCommandFactory.createIntakeEjectC(intakeS));

    spinAndAimTrigger.whileActiveContinuous(
        TurretCommandFactory.createTurretVisionC(limelightS, turretS)
            .alongWith(
                new RunCommand(
                    () -> {
                      LightS.getInstance().requestState(States.Shooting);
                    }))
            .alongWith(
                ShooterCommandFactory.createShooterDistanceSpinupC(() -> limelightS.getFilteredDistance(), shooterS)));

    shooterReadyTrigger.whileActiveContinuous(
        new RunCommand(
            () -> {
              LightS.getInstance().requestState(States.ShooterReady);
            }));

    shooterReadyTrigger.and(distanceInRangeTrigger).whileActiveContinuous(
        new RunCommand(
            () -> {
              LightS.getInstance().requestState(States.ShooterAndDistanceReady);
            }));

    climberS.climberLockedTrigger.whileActiveContinuous(new RunCommand(() -> {
      climberS.stopBack();
      climberS.stopFront();
      climberS.tiltStop();
    }, climberS.linearClimberS, climberS.thriftyClimberS, climberS.tiltClimberS), false);

    climberS.climberLockedTrigger.negate().whileActiveContinuous(
        new RunCommand(
            () -> {
              LightS.getInstance().requestState(States.Climbing);
            }).alongWith(TurretCommandFactory.createTurretClimbLockC(turretS)),
        false);

    // Hold down the right trigger to shoot when ready.
    shootBallTrigger.whileActiveContinuous(
        MidtakeCommandFactory.createMidtakeShootOneC(midtakeS)
            .andThen(MidtakeCommandFactory
                .createMidtakeDefaultC(midtakeS)
                .withInterrupt(midtakeS::getIsArmed)));

    operatorController.start().toggleWhenActive(new StartEndCommand(climberS::unlock, climberS::lock));

    operatorController.back()
        .whileActiveContinuous(TurretCommandFactory.createTurretProfiledVisionC(limelightS, turretS));

    driverController.pov.left()
        .whileActiveContinuous(
            new ConditionalCommand(
                MainCommandFactory.createClimbLockErrorC(),
                ClimberCommandFactory.createClimberLowerC(climberS),
                climberS.climberLockedTrigger));
    driverController.pov.right()
        .whileActiveContinuous(
            new ConditionalCommand(
                MainCommandFactory.createClimbLockErrorC(),
                ClimberCommandFactory.createClimberRaiseC(climberS),
                climberS.climberLockedTrigger));
    operatorController.pov.up()
        .whileActiveContinuous(
            new ConditionalCommand(
                MainCommandFactory.createClimbLockErrorC(),
                ClimberCommandFactory.createClimberExtendBackC(climberS),
                climberS.climberLockedTrigger));
    operatorController.pov.down()
        .whileActiveContinuous(
            new ConditionalCommand(
                MainCommandFactory.createClimbLockErrorC(),
                ClimberCommandFactory.createClimberRetractBackC(climberS),
                climberS.climberLockedTrigger));

    operatorController.x().whileActiveContinuous(
        new ConditionalCommand(
            MainCommandFactory.createClimbLockErrorC(),
            ClimberCommandFactory.createClimberExtendBothMaxC(climberS),
            climberS.climberLockedTrigger));
    operatorController.y().whileActiveContinuous(
        new ConditionalCommand(
            MainCommandFactory.createClimbLockErrorC(),
            ClimberCommandFactory.createClimberExtendFrontC(climberS),
            climberS.climberLockedTrigger));

    operatorController.a().whileActiveContinuous(
        new ConditionalCommand(
            MainCommandFactory.createClimbLockErrorC(),
            ClimberCommandFactory.createClimberRetractFrontC(climberS),
            climberS.climberLockedTrigger));

    operatorController.b().whileActiveContinuous(
        new ConditionalCommand(
            MainCommandFactory.createClimbLockErrorC(),
            ClimberCommandFactory.createClimberTransferC(climberS),
            climberS.climberLockedTrigger));

    shooterS.setDefaultCommand(
        ShooterCommandFactory.createShooterFollowC(() -> 1750, () -> 1750, shooterS));
  }

  public void disableAll() {
    CommandScheduler.getInstance().cancelAll();
    shooterS.stop();
    turretS.stopMotor();
    intakeS.stop();
    intakeS.retract();
    midtakeS.stop();
    drivebaseS.stopAll();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return DrivebaseCommandFactory.createPivotC(2.134, drivebaseS);
    return autoChooser.getSelected();
  }

  public void disabledInit() {
    drivebaseS.setIdleState(IdleMode.kBrake);
  }

  public void enabledInit() {
    drivebaseS.setIdleState(IdleMode.kBrake);
  }

  public void resetPose(Pose2d pose) {
    drivebaseS.resetRobotPose(pose);
    navigationManager.resetPose(pose);
  }

  public void robotPeriodic() {
    // if(spinAndAimChooser.getSelected() == 'M') {
    // limelightS.setDriverMode(true);
    // }
    // if(midtakeManualOverrideTrigger.get()) {
    // midtakeS.setDefaultCommand(midtakeManualC);
    // }

    LightS.getInstance().periodic();

    if (DriverStation.isDisabled()) {
      LightS.getInstance().requestState(States.Disabled);
    }

    if (SmartDashboard.getBoolean("requestPoseReset", false)) {
      resetPose(drivebaseS.START_POSE);
      SmartDashboard.putBoolean("requestPoseReset", false);
    }

    navigationManager.update();

    /* Field2d setup */
    if (RobotBase.isSimulation()) {
      field.setRobotPose(new Pose2d(
          MathUtil.clamp(drivebaseS.getRobotPose().getX(), 0, Units.feetToMeters(54)),
          MathUtil.clamp(drivebaseS.getRobotPose().getY(), 0, Units.feetToMeters(27)),
          drivebaseS.getRobotPose().getRotation()));
    } else {
      field.setRobotPose(drivebaseS.getRobotPose());
    }

    field.getObject("Turret").setPose(field.getRobotPose().transformBy(
        new Transform2d(new Translation2d(), turretS.getRobotToTurretRotation())));

    field.getObject("estRobot").setPose(navigationManager.getEstimatedRobotPose());

    field.getObject("estTurret").setPose(navigationManager.getEstimatedRobotPose().transformBy(
        new Transform2d(new Translation2d(), turretS.getRobotToTurretRotation())));
    SmartDashboard.putNumber("estDist", NomadMathUtil.getDistance(navigationManager.getRobotToHubTransform()));

    field.getObject("turretSetpt").setPose(navigationManager.getEstimatedRobotPose().transformBy(
        new Transform2d(new Translation2d(), navigationManager.getRobotToHubDirection())));

  }
}
