package frc.robot.subsystems;

import static frc.robot.Constants.*;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.SimEncoder;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DrivebaseS extends SubsystemBase implements Loggable {
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());
  private final CANSparkMax frontRight = new CANSparkMax(CAN_ID_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(CAN_ID_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(CAN_ID_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(CAN_ID_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
  private final SlewRateLimiter fwdBackLimiter = new SlewRateLimiter(DRIVEBASE_FWD_BACK_SLEW_LIMIT);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(DRIVEBASE_TURN_SLEW_LIMIT);
  private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(DRIVEBASE_LINEAR_FF[0], DRIVEBASE_LINEAR_FF[1], DRIVEBASE_LINEAR_FF[2]);
  private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(DRIVEBASE_LINEAR_FF[0], DRIVEBASE_LINEAR_FF[1], DRIVEBASE_LINEAR_FF[2]);
  private final AHRS navX = new AHRS(Port.kMXP);

  //Sim stuff
  private DifferentialDrivetrainSim m_driveSim;
  private SimEncoder m_rightEncoder = new SimEncoder();
  private SimEncoder m_leftEncoder = new SimEncoder();
  private Rotation2d m_gyroSim = new Rotation2d();


  /** Creates a new DrivebaseS. */
  public DrivebaseS() {
    frontRight.restoreFactoryDefaults();
    frontLeft.restoreFactoryDefaults();
    backRight.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();
    frontRight.setInverted(true);
    backRight.follow(frontRight, false);
    backLeft.follow(frontLeft, false);

    if (RobotBase.isSimulation()) {
			m_gyroSim = new Rotation2d();
      m_driveSim = new DifferentialDrivetrainSim(
        DRIVEBASE_PLANT,
        DRIVEBASE_GEARBOX,
        DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION,
        DRIVEBASE_TRACKWIDTH,
        DRIVEBASE_WHEEL_DIAMETER / 2.0,
        DRIVEBASE_SIM_ENCODER_STD_DEV);

		}

    odometry.resetPosition(new Pose2d (HUB_CENTER_POSE.getX() - 2, HUB_CENTER_POSE.getY(), Rotation2d.fromDegrees(180)), navX.getRotation2d());

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

  /**
   * Curvature drive method
   * Forward back is from 1 to -1, turn is from 1 to -1
   */
  public void curvatureDrive(double fwdBack, double turn) {
    fwdBack = deadbandJoysticks(fwdBack);
    turn = deadbandJoysticks(turn);
    fwdBack = fwdBackLimiter.calculate(fwdBack);
    turn = turnLimiter.calculate(turn);
    boolean quickTurn = false;
    if (fwdBack == 0) {
      quickTurn = true;
      turn *= 0.25;
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
    frontLeft.setVoltage(left * RobotController.getInputVoltage());
    frontRight.setVoltage(right * RobotController.getInputVoltage());
  }

  public void tankDriveVelocity(double leftVelocityMPS, double rightVelocityMPS) {
    tankDriveVolts(leftFF.calculate(leftVelocityMPS), rightFF.calculate(rightVelocityMPS));
  }

  /**
   * Sets motor speed to 0 when subsystem ends
   */
  public void stopAll() {
    tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotBase.isReal()) {
      odometry.update(navX.getRotation2d(), 
      frontLeft.getEncoder().getPosition() 
      * DRIVEBASE_METERS_PER_WHEEL_ROTATION 
      / DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION,
      frontRight.getEncoder().getPosition()
      * DRIVEBASE_METERS_PER_WHEEL_ROTATION 
      / DRIVEBASE_ENCODER_ROTATIONS_PER_WHEEL_ROTATION);
    }
    else {
      odometry.update(m_gyroSim, m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    }
  }

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

		m_gyroSim = m_driveSim.getHeading();
	}

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public double getLeftVelocity() {
    if(RobotBase.isReal()) {
      return drivebaseEncoderRotationsToMeters(frontLeft.getEncoder().getVelocity() / 60.0);
    }
    else {
      return m_leftEncoder.getVelocity();
    }
  }

  
  public double getRightVelocity() {
    if(RobotBase.isReal()) {
      return drivebaseEncoderRotationsToMeters(frontRight.getEncoder().getVelocity() / 60.0);
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
    odometry.resetPosition(pose, navX.getRotation2d());
    resetEncoders();
  }

  public void resetRobotPose() {
    resetRobotPose(new Pose2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
  }
}
