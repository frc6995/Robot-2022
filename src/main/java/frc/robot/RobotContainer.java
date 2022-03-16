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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  private NetworkTable spinAndAimChooserTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("RobotContainer/Spinup Method");
  @Log(name="Spinup Method")
  private SendableChooser<Character> spinAndAimChooser = new SendableChooser<>();

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
    navigationManager.setVisionEnabledSupplier(()->true);


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
    (list)->{field.getObject("targetRing").setPoses(list);});
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
    turretManualC = TurretCommandFactory.createTurretManualC(()->-operatorController.getLeftX(), turretS);
    //turretS.setDefaultCommand(turretDefaultC);
    turretS.setDefaultCommand(turretManualC);
    
    shooterVisionSpinC = ShooterCommandFactory.createShooterDistanceSpinupC(limelightS::getFilteredDistance, shooterS);
    shooterTestC = new ShooterTestC(shooterS);
    shooterTarmacLineC = ShooterCommandFactory.createShooterDistanceSpinupC(()->3.0, shooterS);
    
    visionSpinAndAimC = MainCommandFactory.createVisionSpinupAndAimC(limelightS, turretS, shooterS);
    odometrySpinAndAimC = new ConditionalCommand(
      MainCommandFactory.createVisionSpinupAndAimC(limelightS, turretS, shooterS),
      MainCommandFactory.createTurretOdometryC(navigationManager, turretS)
      .alongWith(
        MainCommandFactory.createShooterOdometryC(
          navigationManager, shooterS
        )
      ),
      limelightS.hasSteadyTarget);
    

    spinAndAimChooser.setDefaultOption("Vision", 'V');
    spinAndAimChooser.addOption("Odometry", 'O');
    spinAndAimChooser.addOption("Manual/TarmacLine", 'M');
    //shooterS.setDefaultCommand(shooterDefaultC);

    runIntakeC = MainCommandFactory.createIntakeCG(midtakeS, intakeS);
    manualIntakeCG = IntakeCommandFactory.createIntakeRunC(intakeS);

    midtakeDefaultC = MidtakeCommandFactory.createMidtakeDefaultC(midtakeS);
    midtakeFeedC = MidtakeCommandFactory.createMidtakeFeedC(midtakeS);
    midtakeFeedOneC = MidtakeCommandFactory.createMidtakeFeedOneC(midtakeS);
    midtakeManualC = MidtakeCommandFactory.createMidtakeManualC(operatorController::getRightY, midtakeS);
    midtakeS.setDefaultCommand(
        midtakeDefaultC
    );
  }

  public void createTriggers() {
    //turretReadyTrigger = new Trigger(()->turretS.isAtTarget(navigationManager.getRobotToHubDirection()));
    //turretReadyTrigger = new Trigger(()->Math.abs(limelightS.getFilteredXOffset()) < Units.degreesToRadians(5));
    shooterReadyTrigger = new Trigger(shooterS::isAtTarget).and(new Trigger(()->(shooterS.getFrontEncoderSpeed() > 1000)));
    //ballReadyTrigger = new Trigger(midtakeS::getIsArmed);
    //drivebaseStoppedTrigger = new Trigger(drivebaseS::getIsStopped);
    shootButtonTrigger = new Trigger(
      ()->{
        return (operatorController.getRightTriggerAxis() >= 0.5);});
    //distanceInRangeTrigger = new Trigger(
      // ()->{
      //     return NavigationManager.getDistanceInRange(navigationManager.getRobotToHubDistance());});
    distanceInRangeTrigger = new Trigger(()->{return NavigationManager.getDistanceInRange(limelightS.getFilteredDistance());});

    spinAndAimTrigger = new Trigger(()->{return (operatorController.getLeftTriggerAxis() >= 0.5);});

    shootBallTrigger =
      shootButtonTrigger
      //.and(turretReadyTrigger.debounce(0.3))
      .and(shooterReadyTrigger.debounce(0.2))
      //.and(ballReadyTrigger.debounce(0.1))
      //.and(drivebaseStoppedTrigger.debounce(0.2))
      //.and(distanceInRangeTrigger.debounce(0.2))
      //.and(dumpWrongBallTrigger.negate())
    ; 

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
          ()->{
            LightS.getInstance().requestState(States.Intaking);
          }
        )
      )
    );

    driverController.y().whileActiveContinuous(
      IntakeCommandFactory.createIntakeEjectC(intakeS)
    );

    Supplier<Command> spinAndAimSupplier = 
    ()->{
      Command selectedCommand;
      switch(spinAndAimChooser.getSelected()) {
        case 'M' :
          selectedCommand = shooterTarmacLineC;
          break;
        case 'O' :
          selectedCommand = odometrySpinAndAimC;
          break;
        case 'V' :
        default:
          selectedCommand = visionSpinAndAimC;
          break;
      }
      return selectedCommand;
    };
    spinAndAimTrigger.whileActiveContinuous(
      spinAndAimSupplier.get().alongWith(
        new RunCommand(
          ()->{
            LightS.getInstance().requestState(States.Shooting);
          }
        )
      )
    );

    shooterReadyTrigger.whileActiveContinuous(
      new RunCommand(
          ()->{
            LightS.getInstance().requestState(States.ShooterReady);
          }
        )
    );

    shooterReadyTrigger.and(distanceInRangeTrigger).whileActiveContinuous(
      new RunCommand(
          ()->{
            LightS.getInstance().requestState(States.ShooterAndDistanceReady);
          }
        )
    );

    climberS.climberLockedTrigger.whileActiveContinuous(new RunCommand(()->{
      climberS.stopBack();
      climberS.stopFront();
      climberS.tiltStop();
    }), false);

    climberS.climberLockedTrigger.negate().whileActiveContinuous(
      new RunCommand(
          ()->{
            LightS.getInstance().requestState(States.Climbing);
          }
        ).alongWith(TurretCommandFactory.createTurretClimbLockC(turretS)),
        false
    );

    // Hold down the right trigger to shoot when ready.
    shootBallTrigger.whileActiveContinuous(
      MidtakeCommandFactory.createMidtakeShootOneC(midtakeS)
    );
    // dumpWrongBallTrigger.whileActiveContinuous(MainCommandFactory.createWrongBallC(navigationManager, turretS, shooterS));

    // dumpWrongBallTrigger.and(
    //   new Trigger(
    //       ()->{
    //       return (
    //         Math.abs(
    //           turretS.getError(
    //             navigationManager.getRobotToHubDirection()
    //           )
    //         ) > Math.atan2(
    //           Units.feetToMeters(3),
    //           navigationManager.getRobotToHubDistance()
    //         )
    //       );
    //     }
    //   )
    // ).and(shooterReadyTrigger).and(ballReadyTrigger).whenActive(MidtakeCommandFactory.createMidtakeShootOneC(midtakeS));

    operatorController.a().toggleWhenActive(TurretCommandFactory.createTurretClimbLockC(turretS), false);

    operatorController.pov.left()
    .or(operatorController.pov.downLeft())
    .or(operatorController.pov.upLeft())
    .whileActiveContinuous(ClimberCommandFactory.createClimberBackC(climberS));
    operatorController.pov.right()
    .or(operatorController.pov.downRight())
    .or(operatorController.pov.upRight())
    .whileActiveContinuous(ClimberCommandFactory.createClimberForwardC(climberS));
    operatorController.pov.up()
    .or(operatorController.pov.upLeft())
    .or(operatorController.pov.upRight())
    .whileActiveContinuous(ClimberCommandFactory.createClimberExtendBackC(climberS));
    operatorController.pov.down()
    .or(operatorController.pov.downLeft())
    .or(operatorController.pov.downRight())
    .whileActiveContinuous(ClimberCommandFactory.createClimberRetractBackC(climberS));

    operatorController.leftBumper().whileActiveContinuous(ClimberCommandFactory.createClimberExtendFrontC(climberS));
    operatorController.rightBumper().whileActiveContinuous(ClimberCommandFactory.createClimberRetractFrontC(climberS));
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
    return AutoCommandFactory.createTwoBallAutoCG(
        shooterS,
        intakeS,
        midtakeS,
        turretS,
        limelightS,
        drivebaseS);
  }

  public void disabledInit() {
    drivebaseS.setIdleState(IdleMode.kCoast);
  }

  public void enabledInit() {
    drivebaseS.setIdleState(IdleMode.kBrake);
  }

  public void resetPose(Pose2d pose) {
    drivebaseS.resetRobotPose(pose);
    navigationManager.resetPose(pose);
  }

  public void robotPeriodic() {
    if(spinAndAimChooser.getSelected() == 'M') {
      limelightS.setDriverMode(true);
    }
    // if(midtakeManualOverrideTrigger.get()) {
    //   midtakeS.setDefaultCommand(midtakeManualC);
    // }

    LightS.getInstance().periodic();

    if(DriverStation.isDisabled()) {
      LightS.getInstance().requestState(States.Disabled);
    }

    if(SmartDashboard.getBoolean("requestPoseReset", false)) {
      resetPose(drivebaseS.START_POSE);
      SmartDashboard.putBoolean("requestPoseReset", false);
    }

    navigationManager.update();
    
    /*Field2d setup */
    if(RobotBase.isSimulation()) {
      field.setRobotPose(new Pose2d(
        MathUtil.clamp(drivebaseS.getRobotPose().getX(), 0, Units.feetToMeters(54)),
        MathUtil.clamp(drivebaseS.getRobotPose().getY(), 0, Units.feetToMeters(27)),
        drivebaseS.getRobotPose().getRotation()
      ));
    }
    else {
      field.setRobotPose(drivebaseS.getRobotPose());
    }


    field.getObject("Turret").setPose(field.getRobotPose().transformBy(
      new Transform2d(new Translation2d(), turretS.getRobotToTurretRotation())
      )
    );

    field.getObject("estRobot").setPose(navigationManager.getEstimatedRobotPose());

    field.getObject("estTurret").setPose(navigationManager.getEstimatedRobotPose().transformBy(
      new Transform2d(new Translation2d(), turretS.getRobotToTurretRotation())));
    SmartDashboard.putNumber("estDist", NomadMathUtil.getDistance(navigationManager.getRobotToHubTransform()));

    field.getObject("turretSetpt").setPose(navigationManager.getEstimatedRobotPose().transformBy(
      new Transform2d(new Translation2d(), navigationManager.getRobotToHubDirection())));
    
  }
}
