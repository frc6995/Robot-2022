package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommandFactory;
import frc.robot.commands.MidtakeCommandFactory;
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
import frc.robot.util.interpolation.ShooterInterpolatingTable;
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
  private Command turretManualC;
  private Command turretTurningC;
  private Command turretAimC;
  private Command shooterSpinC;
  private Command runMidtakeC;
  private Command runIntakeC;
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
   * Instantiate the driver and operator controllers
   */
  private void createControllers() {
    driverController = new CommandXboxController(Constants.USB_PORT_DRIVER_CONTROLLER);
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
    odometryManager = new OdometryManager(
      drivebaseS::getRobotPose,
      drivebaseS::getRotation2d,
      drivebaseS::getChassisSpeeds,
      turretS::getRotation2d,
      turretS::setTransformVelocity
    );
    odometryManager.setVisionEnabled(false);
    limelightS = new LimelightS(odometryManager,
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

    runTurretC = TurretCommandFactory.createTurretManualC(
      driverController::getRightX, turretS);

    turretAimC = TurretCommandFactory.createTurretFollowC(odometryManager::getRotationOffset, turretS);
    turretManualC = TurretCommandFactory.createTurretManualC(driverController::getRightX, turretS);
    turretS.setDefaultCommand(turretAimC);
    
    shooterSpinC = ShooterCommandFactory.createShooterFollowC(
      ()->{
        return ShooterInterpolatingTable.get(odometryManager.getDistanceToCenter()).frontWheelRpm;
      },
      ()->{
        return ShooterInterpolatingTable.get(odometryManager.getDistanceToCenter()).backWheelRpm;
      },
      shooterS);
    turretTurningC = TurretCommandFactory.createTurretTurnC(0, turretS);
    shooterTestC = new ShooterTestC(shooterS);
    runIntakeC = IntakeCommandFactory.getIntakeRunCommand(intakeS);
    runMidtakeC = MidtakeCommandFactory.getMidtakeRunCommand(midtakeS);
    //shooterS.setDefaultCommand(ShooterCommandFactory.createShooterIdleC(shooterS));
    SmartDashboard.putData(new InstantCommand(turretS::resetEncoder).withName("Turret Reset"));
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
  
    
    targetDistanceInRangeTrigger = new Trigger(odometryManager::getDistanceInRange);
    turretReadyTrigger = new Trigger(turretS::isAtTarget);
    shooterReadyTrigger = new Trigger(shooterS::isAtTarget);
    ballReadyTrigger = new Trigger(midtakeS::getIsTopBeamBroken); // TODO fix this logic

    //targetDistanceInRangeTrigger.whileActiveContinuous(shooterSpinC);

    // shootBallTrigger = targetDistanceInRangeTrigger.and(turretReadyTrigger).and(shooterReadyTrigger).and(ballReadyTrigger);

    // shootBallTrigger.whenActive(runMidtake);
    driverController.b().whileActiveOnce(runMidtakeC);
    driverController.x().whileActiveOnce(shooterTestC);
    driverController.y().toggleWhenActive(turretManualC);
    driverController.a().whileActiveOnce(runIntakeC);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(RobotBase.isSimulation()) {
      return DrivebaseCommandFactory.createRamseteC(
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(),
          List.of(),
          new Pose2d(2, 1, new Rotation2d()), 
          Constants.TRAJECTORY_CONFIG).transformBy(new Transform2d(new Pose2d(), odometryManager.getCurrentRobotPose())),
          odometryManager,
        drivebaseS).andThen(drivebaseS::stopAll, drivebaseS);
    }
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

    odometryManager.periodic();

    /*Field2d setup */
    if(RobotBase.isSimulation()) {
      field.setRobotPose(new Pose2d(
        MathUtil.clamp(odometryManager.getCurrentRobotPose().getX(), 0, Units.feetToMeters(54)),
        MathUtil.clamp(odometryManager.getCurrentRobotPose().getY(), 0, Units.feetToMeters(27)),
        odometryManager.getCurrentRobotPose().getRotation()
      ));
    }
    else {
      field.setRobotPose(odometryManager.getCurrentRobotPose());
    }


    

    field.getObject("estRobot").setPose(
      odometryManager.getEstimatedRobotPose());

    field.getObject("target").setPose(odometryManager.getCurrentRobotPose().transformBy(odometryManager.getRobotToHub()));
      
    field.getObject("Turret").setPose(odometryManager.getCurrentRobotPose().transformBy(
        odometryManager.getRobotToTurret()
      )
    );

    field.getObject("camera").setPose(odometryManager.getCurrentRobotPose().transformBy(odometryManager.getRobotToCamera()));
    
  }
}
