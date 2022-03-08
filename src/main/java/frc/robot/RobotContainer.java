package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.subsystems.ClimberS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LimelightS;
import frc.robot.subsystems.MidtakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.TurretS;
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
  private ClimberS climberS;
  private LimelightS limelightS;

  // Command
  private Command xboxDriveCommand;
  private Command runTurretC;
  private Command turretManualC;
  private Command turretAimC;
  private Command shooterSpinC;
  private Command visionSpinAndAimC;
  private Command midtakeFeedC;
  private Command midtakeFeedOneC;
  private Command midtakeIndexCG;
  private Command runIntakeC;
  private Command climberRetractTwoC;
  private Command climberExtendTwoC;
  @Log
  @Config
  private Command shooterTestC;

  private Trigger shooterReadyTrigger;
  private Trigger turretReadyTrigger;
  private Trigger ballReadyTrigger;

  private Trigger shootBallTrigger;


  public RobotContainer() {
    createControllers();
    createSubsystems();
    createCommands();
    createTriggers();
    configureButtonBindings();
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
    climberS = new ClimberS();
    limelightS = new LimelightS(
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



    visionSpinAndAimC = MainCommandFactory.createVisionSpinupAndAimC(limelightS, turretS, shooterS);

    turretAimC = TurretCommandFactory.createTurretVisionC(limelightS, turretS);
    turretManualC = TurretCommandFactory.createTurretManualC(()->-operatorController.getLeftX(), turretS);
    turretS.setDefaultCommand(turretManualC);
    
    shooterSpinC = ShooterCommandFactory.createShooterDistanceSpinupC(limelightS::getFilteredDistance, shooterS);
    shooterTestC = new ShooterTestC(shooterS);
    runIntakeC = IntakeCommandFactory.createIntakeToggleRunC(intakeS);
    midtakeIndexCG = MidtakeCommandFactory.createMidtakeIndexCG(midtakeS);
    midtakeS.setDefaultCommand(midtakeIndexCG);
    midtakeFeedC = MidtakeCommandFactory.createMidtakeFeedC(midtakeS);
    midtakeFeedOneC = MidtakeCommandFactory.createMidtakeFeedOneC(midtakeS);

    climberExtendTwoC = ClimberCommandFactory.createClimberExtendArmTwoC(climberS);
    climberRetractTwoC = ClimberCommandFactory.createClimberRetractArmTwoC(climberS);
  }

  public void createTriggers() {
    turretReadyTrigger = new Trigger(()->{return Math.abs(limelightS.getFilteredXOffset()) < 0.1;});
    shooterReadyTrigger = new Trigger(shooterS::isAtTarget);
    ballReadyTrigger = new Trigger(()->{return midtakeS.getStoredCargo() > 0;});

    shootBallTrigger =
      driverController.rightBumper()
      .and(turretReadyTrigger)
      .and(shooterReadyTrigger.debounce(0.2, DebounceType.kBoth))
      .and(ballReadyTrigger);
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
    driverController.a().toggleWhenActive(runIntakeC);
    driverController.b().whileActiveOnce(IntakeCommandFactory.createIntakeEjectC(intakeS));
    driverController.x().whileActiveOnce(
      MidtakeCommandFactory.createMidtakeReverseC(midtakeS)
      .alongWith(IntakeCommandFactory.createIntakeEjectC(intakeS)));
    driverController.rightBumper().whenActive(
      TurretCommandFactory.createTurretAdjustC( Units.degreesToRadians(-5),
      turretS)
    );
    operatorController.leftBumper().whenActive(
      TurretCommandFactory.createTurretAdjustC( Units.degreesToRadians(5),
      turretS)
    );
    
    //driverController.rightBumper() does (midtakeFeedOneC) when ready to fire.
    shootBallTrigger.whenActive(midtakeFeedOneC);
    //operatorController.a().toggleWhenActive(new ShooterTestC(shooterS));
    new Trigger(()->{return (operatorController.getRightTriggerAxis() >= 0.5);}).whileActiveContinuous(turretAimC.alongWith(shooterTestC));
    //operatorController.a().toggleWhenActive(shooterSpinC);
    // //driverController.leftBumper() feeds whenever the shooter RPM is above 1000;
    operatorController.y()/*.and(new Trigger(()->(shooterS.getFrontEncoderSpeed() > 1000)))*/.whileActiveOnce(midtakeFeedC).whenInactive(MidtakeCommandFactory.createMidtakeStopC(midtakeS));
    
    operatorController.back().whileActiveOnce(new RunCommand(climberS::spinTwo, climberS)).whenInactive(new InstantCommand(climberS::stopMotorArmTwo, climberS));
    operatorController.start().whileActiveOnce(climberRetractTwoC);
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

  public void robotPeriodic() {
    
    // /*Field2d setup */
    // if(RobotBase.isSimulation()) {
    //   field.setRobotPose(new Pose2d(
    //     MathUtil.clamp(drivebaseS.getRobotPose().getX(), 0, Units.feetToMeters(54)),
    //     MathUtil.clamp(drivebaseS.getRobotPose().getY(), 0, Units.feetToMeters(27)),
    //     drivebaseS.getRobotPose().getRotation()
    //   ));
    // }
    // else {
    //   field.setRobotPose(drivebaseS.getRobotPose());
    // }


    // field.getObject("Turret").setPose(field.getRobotPose().transformBy(
    //   new Transform2d(new Translation2d(), turretS.getRobotToTurretRotation())

    //   )
    // );

    
  }
}
