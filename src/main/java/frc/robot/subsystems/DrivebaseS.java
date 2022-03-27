package frc.robot.subsystems;

import static frc.robot.Constants.CAN_ID_BACK_LEFT_DRIVE_MOTOR;
import static frc.robot.Constants.CAN_ID_BACK_RIGHT_DRIVE_MOTOR;
import static frc.robot.Constants.CAN_ID_FRONT_LEFT_DRIVE_MOTOR;
import static frc.robot.Constants.CAN_ID_FRONT_RIGHT_DRIVE_MOTOR;
import static frc.robot.Constants.DRIVEBASE_DEADBAND;
import static frc.robot.Constants.DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION;
import static frc.robot.Constants.DRIVEBASE_FWD_BACK_SLEW_LIMIT;
import static frc.robot.Constants.DRIVEBASE_GEARBOX;
import static frc.robot.Constants.DRIVEBASE_KINEMATICS;
import static frc.robot.Constants.DRIVEBASE_LINEAR_FF_MPS;
import static frc.robot.Constants.DRIVEBASE_METERS_PER_WHEEL_ROTATION;
import static frc.robot.Constants.DRIVEBASE_P;
import static frc.robot.Constants.DRIVEBASE_PLANT;
import static frc.robot.Constants.DRIVEBASE_SIM_ENCODER_STD_DEV;
import static frc.robot.Constants.DRIVEBASE_TRACKWIDTH;
import static frc.robot.Constants.DRIVEBASE_TURN_SLEW_LIMIT;
import static frc.robot.Constants.HUB_CENTER_POSE;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
//import frc.robot.util.pose.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SimEncoder;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DrivebaseS extends SubsystemBase implements Loggable {
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());
  //DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Constants.DRIVEBASE_TRACKWIDTH);
  private final CANSparkMax frontRight = new CANSparkMax(CAN_ID_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(CAN_ID_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(CAN_ID_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(CAN_ID_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
  private final SlewRateLimiter fwdBackLimiter = new SlewRateLimiter(DRIVEBASE_FWD_BACK_SLEW_LIMIT);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(DRIVEBASE_TURN_SLEW_LIMIT);
  public final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(DRIVEBASE_LINEAR_FF_MPS[0], DRIVEBASE_LINEAR_FF_MPS[1], DRIVEBASE_LINEAR_FF_MPS[2]);
  public final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(DRIVEBASE_LINEAR_FF_MPS[0], DRIVEBASE_LINEAR_FF_MPS[1], DRIVEBASE_LINEAR_FF_MPS[2]);
  public final PIDController angularPID = new PIDController(3, 0, 0);
  public final PIDController leftPID = new PIDController(DRIVEBASE_P, 0, 0);
  public final PIDController rightPID = new PIDController(DRIVEBASE_P, 0, 0);
  private final AHRS navX = new AHRS(Port.kMXP);
  public final RamseteController ramseteController = new RamseteController();
  public final Pose2d START_POSE = new Pose2d (6.67436, 2.651410, Rotation2d.fromDegrees(-155.055217));
  @Log
  public final Mechanism2d drivebaseTilt = new Mechanism2d(36, 36);
  private final MechanismRoot2d drivebaseTiltRoot = drivebaseTilt.getRoot("drivebaseRoot", 12, 18);
  private final MechanismLigament2d drivebaseTiltLigament = drivebaseTiltRoot.append(new MechanismLigament2d("drivebase", 25, 0));
  //Sim stuff
  private DifferentialDrivetrainSim m_driveSim;
  @Log(methodName = "getPosition", name = "getSimRightPosition")
  private SimEncoder m_rightEncoder = new SimEncoder();
  @Log(methodName = "getPosition", name = "getSimLeftPosition")
  private SimEncoder m_leftEncoder = new SimEncoder();
  private Rotation2d m_simHeading = new Rotation2d();


  /** Creates a new DrivebaseS. */
  public DrivebaseS() {
    frontRight.restoreFactoryDefaults();
    frontLeft.restoreFactoryDefaults();
    frontRight.getEncoder().setPositionConversionFactor(DRIVEBASE_METERS_PER_WHEEL_ROTATION 
    / DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION);
    frontRight.getEncoder().setVelocityConversionFactor(DRIVEBASE_METERS_PER_WHEEL_ROTATION 
    / DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION / 60);
    frontLeft.getEncoder().setPositionConversionFactor(DRIVEBASE_METERS_PER_WHEEL_ROTATION 
    / DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION);
    frontLeft.getEncoder().setVelocityConversionFactor(DRIVEBASE_METERS_PER_WHEEL_ROTATION 
    / DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION / 60);
    backRight.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();
    frontRight.setIdleMode(IdleMode.kCoast);
    frontLeft.setIdleMode(IdleMode.kCoast);
    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    backRight.follow(frontRight, false);
    backLeft.follow(frontLeft, false);

    angularPID.setTolerance(Units.degreesToRadians(5));
    angularPID.enableContinuousInput(-Math.PI, Math.PI);


    SmartDashboard.putBoolean("requestPoseReset", false);

    if (RobotBase.isSimulation()) {
      m_driveSim = new DifferentialDrivetrainSim(
        DRIVEBASE_PLANT,
        DRIVEBASE_GEARBOX,
        DRIVEBASE_METERS_PER_WHEEL_ROTATION,
        DRIVEBASE_TRACKWIDTH,
        1 / 2.0, // wheel radius is half of an encoder position unit.
        DRIVEBASE_SIM_ENCODER_STD_DEV);
		}
    resetRobotPose(START_POSE);
    
  }

  public void burnFlash() {
    frontRight.burnFlash();
    frontLeft.burnFlash();
    backRight.burnFlash();
    backLeft.burnFlash();
  }



  public void pivot(double targetAngle) {
    double omega = angularPID.calculate(getEstimatedHeading().getRadians(), targetAngle);
    SmartDashboard.putNumber("driveOmega", omega);
    DifferentialDriveWheelSpeeds speeds = Constants.DRIVEBASE_KINEMATICS.toWheelSpeeds(new ChassisSpeeds(0, 0, omega));
    SmartDashboard.putNumber("driveRightVelo", speeds.rightMetersPerSecond);
    double rightVoltage = rightFF.calculate(speeds.rightMetersPerSecond);
    double leftVoltage = leftFF.calculate(speeds.leftMetersPerSecond);
    tankDriveVolts(leftVoltage, rightVoltage);
  }

  public boolean isPivotAtTarget() {
    return angularPID.atSetpoint();
  }

  /**
   * Creates a deadband for the drivebase joystick
   */
  public double deadbandJoysticks(double value) {
    if (Math.abs(value) < DRIVEBASE_DEADBAND) {
      value = 0;
    }
    return value;
  }
  

  @Log(methodName = "getRadians", name = "gyroHeading")
  public Rotation2d getRotation2d() {
    if(RobotBase.isReal()) {
      return navX.getRotation2d();
    }
    else {
      return m_simHeading;
    }
  }

  @Log(methodName = "getRadians")
  public Rotation2d getEstimatedHeading() {
    return odometry.getPoseMeters().getRotation();
  }

  /**
   * Curvature drive method
   * Forward back is from 1 to -1, turn is from 1 to -1
   */
  public void curvatureDrive(double fwdBack, double turn) {
    fwdBack = MathUtil.applyDeadband(fwdBack, 0.02);
    turn = MathUtil.applyDeadband(turn, 0.02);
    fwdBack = fwdBackLimiter.calculate(fwdBack);
    turn = turnLimiter.calculate(turn);
    boolean quickTurn = false;
    if (fwdBack == 0) {
      quickTurn = true;
      turn *= 0.35;
    }
    WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(fwdBack, turn, quickTurn);
    tankDrive(speeds.left, speeds.right);
  }

  /**
   * makes robot drive with both wheels going in the same direction
   * 
   * @param left
   * @param right
   */
  public void tankDrive(double left, double right) {
    SmartDashboard.putNumber("leftSpeed", left);
    SmartDashboard.putNumber("rightSpeed", right);
    frontLeft.setVoltage(left * 6.5);
    frontRight.setVoltage(right * 6.5);
  }

  public void tankDriveVelocity(double leftVelocityMPS, double rightVelocityMPS) {
    SmartDashboard.putNumber("leftVelo", leftVelocityMPS);
    SmartDashboard.putNumber("rightVelo", rightVelocityMPS);
    tankDriveVolts(
      leftFF.calculate(leftVelocityMPS) + leftPID.calculate(getLeftVelocity(), leftVelocityMPS),
      rightFF.calculate(rightVelocityMPS) + rightPID.calculate(getRightVelocity(), rightVelocityMPS));
  }

  /**
   * Sets motor speed to 0 when subsystem ends
   */
  public void stopAll() {
    tankDriveVelocity(0, 0);
  }

  public boolean getIsStopped() {
    return Math.abs(getLeftVelocity()) < 0.1
    && Math.abs(getRightVelocity()) < 0.1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotBase.isReal()) {
      odometry.update(getRotation2d(), 
      frontLeft.getEncoder().getPosition(),
      frontRight.getEncoder().getPosition()
      );
    }
    else {
      odometry.update(getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    }

    
    drivebaseTiltLigament.setAngle(navX.getPitch());


  }

  @Override
  public void simulationPeriodic() {
		// Set the inputs to the system. Note that we need to convert
		// the [-1, 1] PWM signal to voltage by multiplying it by the
		// robot controller voltage.
		m_driveSim.setInputs(frontLeft.getAppliedOutput(), frontRight.getAppliedOutput());

		// Advance the model by 20 ms. Note that if you are running this
		// subsystem in a separate thread or have changed the nominal timestep
		// of TimedRobot, this value needs to match it.
		m_driveSim.update(0.02);

		// Update all of our sensors.
		m_leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
		m_leftEncoder.setVelocity(m_driveSim.getLeftVelocityMetersPerSecond());
		m_rightEncoder.setPosition(m_driveSim.getRightPositionMeters());
		m_rightEncoder.setVelocity(m_driveSim.getRightVelocityMetersPerSecond());

		m_simHeading = m_driveSim.getHeading();
	}

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DRIVEBASE_KINEMATICS.toChassisSpeeds(getWheelSpeeds());
  }

  @Log
  public double getLeftVelocity() {
    if(RobotBase.isReal()) {
      return frontLeft.getEncoder().getVelocity();
    }
    else {
      return m_leftEncoder.getVelocity();
    }
  }

  @Log
  public double getRightVelocity() {
    if(RobotBase.isReal()) {
      return frontRight.getEncoder().getVelocity();
    }
    else {
      return m_rightEncoder.getVelocity();
    }
  }

  public void resetEncoders() {
    if(RobotBase.isReal())
    {
      frontLeft.getEncoder().setPosition(0);
      frontRight.getEncoder().setPosition(0);
    }
    else {
      m_leftEncoder.setPosition(0);
      m_leftEncoder.setVelocity(0);
      m_rightEncoder.setPosition(0);
      m_rightEncoder.setVelocity(0);
    }


  }
  
  @Log(methodName = "toString")
  public Pose2d getRobotPose() {
    return odometry.getPoseMeters();
  }

  public void resetRobotPose(Pose2d pose) {
    
    if(RobotBase.isSimulation()) {
      //odometry.resetPosition(pose);
      odometry.resetPosition(pose, getRotation2d());
    }
    else {
      //odometry.resetPosition(pose);
      odometry.resetPosition(pose, getRotation2d());
    }


    resetEncoders(); 
       
  }

  public void resetRobotPose() {
    resetRobotPose(new Pose2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    if(RobotBase.isSimulation()) {
      if(Math.abs(leftVolts) > Constants.DRIVEBASE_LINEAR_FF_MPS[0]) {
        leftVolts -= Math.copySign(Constants.DRIVEBASE_LINEAR_FF_MPS[0], leftVolts);
      }
      if(Math.abs(rightVolts) > Constants.DRIVEBASE_LINEAR_FF_MPS[0]) {
        rightVolts -= Math.copySign(Constants.DRIVEBASE_LINEAR_FF_MPS[0], rightVolts);
      }
    }
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
  }

  public void setIdleState(IdleMode mode) {
    frontLeft.setIdleMode(mode);
    frontRight.setIdleMode(mode);
    backLeft.setIdleMode(mode);
    backRight.setIdleMode(mode);
  }
}
