package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The shooter subsystem.
 * This handles the two sets of wheels that shoot the cargo into the hub
 * 
 * @author Noah Kim
 */
public class ShooterS extends SubsystemBase implements Loggable {
  private final CANSparkMax frontSparkMax = new CANSparkMax(Constants.CAN_ID_FRONT_SHOOTER_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backSparkMax = new CANSparkMax(Constants.CAN_ID_BACK_SHOOTER_MOTOR, MotorType.kBrushless);
  private RelativeEncoder frontEncoder;
  @Log
  private double lastFrontEncoderPosition = 0;
  @Log
  private double frontEncoderVelocityRPM = 0;
  private RelativeEncoder backEncoder;
  @Log
  private double lastBackEncoderPosition = 0;
  @Log
  private double backEncoderVelocityRPM = 0;

  private PIDController frontPID = new PIDController(Constants.SHOOTER_FRONT_P, 0, 0);
  private PIDController backPID = new PIDController(Constants.SHOOTER_BACK_P, 0, 0);
  private SimpleMotorFeedforward frontFF = new SimpleMotorFeedforward(
    Constants.SHOOTER_FRONT_FF[0],
    Constants.SHOOTER_FRONT_FF[1],
    Constants.SHOOTER_FRONT_FF[2]);
  private FlywheelSim frontSim = new FlywheelSim(
    LinearSystemId.identifyVelocitySystem(
      Constants.SHOOTER_FRONT_FF[1], 
      Constants.SHOOTER_FRONT_FF[2]), 
      DCMotor.getNEO(1), 1);
  private SimpleMotorFeedforward backFF = new SimpleMotorFeedforward(
    Constants.SHOOTER_BACK_FF[0],
    Constants.SHOOTER_BACK_FF[1],
    Constants.SHOOTER_BACK_FF[2]);
    private FlywheelSim backSim = new FlywheelSim(
      LinearSystemId.identifyVelocitySystem(
        Constants.SHOOTER_BACK_FF[1], 
        Constants.SHOOTER_BACK_FF[2]), 
        DCMotor.getNEO(1), 1);

  /** Creates a new ShooterS. */
  public ShooterS() {
    frontSparkMax.restoreFactoryDefaults();
    backSparkMax.restoreFactoryDefaults();
    frontSparkMax.setClosedLoopRampRate(6);
    backSparkMax.setClosedLoopRampRate(6);
    frontSparkMax.setSmartCurrentLimit(20, 30, 0);
    backSparkMax.setSmartCurrentLimit(20, 30, 0);

    frontEncoder = frontSparkMax.getEncoder();
    backEncoder = backSparkMax.getEncoder();


    frontPID.setTolerance(Constants.SHOOTER_PID_ERROR, 0);
    backPID.setTolerance(Constants.SHOOTER_PID_ERROR, 0);

    frontPID.setIntegratorRange(0, 0);
    backPID.setIntegratorRange(0, 0);
  }

  /**
   * When running in manual mode, add a deadband
   * 
   * @param value The input value
   * @return The deadbanded value
   */
  public double deadbandJoystick(double value) {
    if (Math.abs(value) < Constants.DRIVEBASE_DEADBAND) {
      value = 0;
    }

    return value;
  }

  /**
   * Gets the front encoder speed
   * 
   * @return The velocity of the front motor
   */
  @Log
  public double getFrontEncoderSpeed() {
    return frontEncoderVelocityRPM;

  }

  /**
   * Gets the back encoder speed
   * 
   * @return The velocity of the back motor
   */
  @Log
  public double getBackEncoderSpeed() {
    return backEncoderVelocityRPM;
  }

  /**
   * Sets the speed of the front motor
   * 
   * @param speed Speed value for front motor
   */
  public void setFrontSpeed(double speed) {
    speed = deadbandJoystick(speed);
    frontSparkMax.setVoltage(speed * RobotController.getInputVoltage());
  }

  /**
   * Sets the speed of the back motor
   * 
   * @param speed Speed value for the back motor
   */
  public void setBackSpeed(double speed) {
    speed = deadbandJoystick(speed);
    backSparkMax.setVoltage(speed * RobotController.getInputVoltage());
  }

  /**
   * sets the speed of the front motor using PID from -1 to 1
   * 
   * @param frontTargetRPM The target RPM of the front motor
   */
  public void pidFrontSpeed(double frontTargetRPM) {
    SmartDashboard.putNumber("frontTargetRPM", frontTargetRPM);
    frontSparkMax.setVoltage(
        frontPID.calculate(getFrontEncoderSpeed() / 60.0, frontTargetRPM / 60.0)
            + frontFF.calculate(frontTargetRPM / 60.0));

  }

  /**
   * sets the speed of the back motor using PID from -1 to 1
   * 
   * @param backTargetRPM The target RPM of the front motor
   */
  public void pidBackSpeed(double backTargetRPM) {
    SmartDashboard.putNumber("backTargetRPM", backTargetRPM);
    backSparkMax.setVoltage(
        backPID.calculate(getBackEncoderSpeed() / 60.0, backTargetRPM / 60.0) + backFF.calculate(backTargetRPM / 60.0));
  }

  /**
   * stops both motors
   */
  public void stop() {
    setFrontSpeed(0);
    setBackSpeed(0);
  }

  /**
   * gets the velocity error of the front motor
   * 
   * @return The velocity error of the front motor
   */
  @Log
  public double getFrontError() {
    return frontPID.getVelocityError();
  }

  /**
   * gets the velocity error of teh back motor
   * 
   * @return The velocity error of teh back motor
   */
  @Log
  public double getBackError() {
    return backPID.getVelocityError();
  }

  /**
   * determines whether the front motor is at the target RPM
   * 
   * @return True if the front motor is at the target RPM
   */
  @Log
  public boolean isFrontAtTarget() {
    return frontPID.atSetpoint();
  }

  /**
   * determines whether the back motor as at the target RPM
   * 
   * @return True if the back motor is at the target RPM
   */
  @Log
  public boolean isBackAtTarget() {
    return backPID.atSetpoint();
  }

  /**
   * determines whether both motors are at the target RPMs.
   * 
   * @return True if both motors are at the target RPMs
   */
  @Log
  public boolean isAtTarget() {
    return isBackAtTarget() && isFrontAtTarget();
  }

  /**
   * Converts a distance in feet into the proper shooter RPM
   * 
   * @param distance the distance to the target in feet (measured horizontally
   *                 from the Limelight to the vision ring)
   * @return the shooter RPM
   */

  public static double getSpeedForDistance(double distance, boolean isBackWheel) {
    int index = Math.max(Math.min(Constants.SHOOTER_DISTANCES.length-2, getIndexForDistance(distance, Constants.SHOOTER_DISTANCES)), 0);
    double rpm = calcSpeed(index, index+1, distance, Constants.SHOOTER_DISTANCES, Constants.SHOOTER_SPEEDS, isBackWheel);
    return rpm; 
  }

  /**
   * Finds the index of the highest distance in the provided array that is less
   * than the provided distance.
   * 
   * @param distance       the distance
   * @param DISTANCES_FEET the distance array
   * @return the index
   */

  public static int getIndexForDistance(double distance, double[] DISTANCES_FEET) {
    int index = 0;
    for (int i = 0; i < DISTANCES_FEET.length; i++) {
      if (distance > DISTANCES_FEET[i]) {
        index = i;
      }
    }
    return index;
  }

   /**
   * Given two points in an array of doubles and an x point between them,
   * draw a line between them and find the appropriate y value
   * 
   * @param smallerIndex
   * @param biggerIndex
   * @param distance
   * @param DISTANCES_FEET
   * @param RPMS
   * @return
   */
  public static double calcSpeed(int smallerIndex, int biggerIndex, double distance, double[] DISTANCES_FEET, double[][] RPMS, boolean isBackWheel) {
    double smallerRPM = RPMS[Math.max(smallerIndex, 0)][isBackWheel ? 1 : 0];
    double biggerRPM = RPMS[Math.min(biggerIndex, RPMS.length-1)][isBackWheel ? 1 : 0] + 0.0001; //add a tiny amount to avoid NaN if distance is out of range
    double smallerDistance = DISTANCES_FEET[Math.max(smallerIndex, 0)];
    double biggerDistance = DISTANCES_FEET[Math.min(biggerIndex, DISTANCES_FEET.length - 1)] + 0.0001;
    double newRPM = ((biggerRPM - smallerRPM) / (biggerDistance - smallerDistance) * (distance - smallerDistance))
        + smallerRPM;

    return newRPM;
  }

  @Override
  public void periodic() {
    double frontEncoderPosition = 0;
    double backEncoderPosition = 0;
    if(Robot.isReal()) {
      frontEncoderPosition = frontEncoder.getPosition();
      backEncoderPosition = backEncoder.getPosition();
      frontEncoderVelocityRPM = frontEncoder.getVelocity();
      backEncoderVelocityRPM = backEncoder.getVelocity();
      // frontEncoderVelocityRPM = (frontEncoderPosition - lastFrontEncoderPosition) * 60.0 /*seconds/minute*/ / 0.02 /*seconds dt*/;
      // backEncoderVelocityRPM = (backEncoderPosition - lastBackEncoderPosition) * 60.0 /*seconds/minute*/ / 0.02 /*seconds dt*/;
      lastFrontEncoderPosition = frontEncoderPosition;
      lastBackEncoderPosition = backEncoderPosition;
    }
    else {
    }
  }

  @Override
  public void simulationPeriodic() {
    frontSim.setInput(frontSparkMax.getAppliedOutput() - 
      (Math.signum(frontSparkMax.getAppliedOutput()) * Constants.SHOOTER_FRONT_FF[0]));
    backSim.setInput(backSparkMax.getAppliedOutput() - 
      (Math.signum(backSparkMax.getAppliedOutput()) * Constants.SHOOTER_BACK_FF[0]));
    
    frontSim.update(0.02);
    backSim.update(0.02);

    frontEncoderVelocityRPM = frontSim.getAngularVelocityRPM();
    backEncoderVelocityRPM = backSim.getAngularVelocityRPM();
  }
}
