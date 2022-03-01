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
import static frc.robot.Constants.*;
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
  private final CANSparkMax frontSparkMax = new CANSparkMax(CAN_ID_FRONT_SHOOTER_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backSparkMax = new CANSparkMax(CAN_ID_BACK_SHOOTER_MOTOR, MotorType.kBrushless);
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

  private PIDController frontPID = new PIDController(SHOOTER_FRONT_P, 0, 0);
  private PIDController backPID = new PIDController(SHOOTER_BACK_P, 0, 0);
  private SimpleMotorFeedforward frontFF = new SimpleMotorFeedforward(
    SHOOTER_FRONT_FF[0],
    SHOOTER_FRONT_FF[1],
    SHOOTER_FRONT_FF[2]);
  private FlywheelSim frontSim = new FlywheelSim(
    LinearSystemId.identifyVelocitySystem(
      SHOOTER_FRONT_FF[1], 
      SHOOTER_FRONT_FF[2]), 
      DCMotor.getNEO(1), 1);
  private SimpleMotorFeedforward backFF = new SimpleMotorFeedforward(
    SHOOTER_BACK_FF[0],
    SHOOTER_BACK_FF[1],
    SHOOTER_BACK_FF[2]);
    private FlywheelSim backSim = new FlywheelSim(
      LinearSystemId.identifyVelocitySystem(
        SHOOTER_BACK_FF[1], 
        SHOOTER_BACK_FF[2]), 
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


    frontPID.setTolerance(SHOOTER_PID_ERROR, 0);
    backPID.setTolerance(SHOOTER_PID_ERROR, 0);

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
    if (Math.abs(value) < DRIVEBASE_DEADBAND) {
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
    return frontEncoder.getVelocity();

  }

  /**
   * Gets the back encoder speed
   * 
   * @return The velocity of the back motor
   */
  @Log
  public double getBackEncoderSpeed() {
    return backEncoder.getVelocity();
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
        /*frontPID.calculate(getFrontEncoderSpeed() / 60.0, frontTargetRPM / 60.0)
            + */frontFF.calculate(frontTargetRPM / 60.0));

  }

  /**
   * sets the speed of the back motor using PID from -1 to 1
   * 
   * @param backTargetRPM The target RPM of the front motor
   */
  public void pidBackSpeed(double backTargetRPM) {
    SmartDashboard.putNumber("backTargetRPM", backTargetRPM);
    backSparkMax.setVoltage(
        /*backPID.calculate(getBackEncoderSpeed() / 60.0, backTargetRPM / 60.0) + */backFF.calculate(backTargetRPM / 60.0));
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

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    frontSim.setInput(frontSparkMax.getAppliedOutput() - 
      (Math.signum(frontSparkMax.getAppliedOutput()) * SHOOTER_FRONT_FF[0]));
    backSim.setInput(backSparkMax.getAppliedOutput() - 
      (Math.signum(backSparkMax.getAppliedOutput()) * SHOOTER_BACK_FF[0]));
    
    frontSim.update(0.02);
    backSim.update(0.02);

    frontEncoderVelocityRPM = frontSim.getAngularVelocityRPM();
    backEncoderVelocityRPM = backSim.getAngularVelocityRPM();
  }
}
