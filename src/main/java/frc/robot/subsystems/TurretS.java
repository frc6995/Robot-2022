package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.*;

import javax.swing.text.Position;

import frc.robot.util.SimEncoder;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The turret subsystem. It uses a PID controller to control the turret position.
 * 
 * CCW POSITIVE
 * 
 * @author Benjamin Su
 * @author Noah Kim
 */
public class TurretS extends SubsystemBase implements Loggable {
  @Log(methodName = "getAppliedOutput")
  private CANSparkMax sparkMax = new CANSparkMax(CAN_ID_TURRET, MotorType.kBrushless);
  private RelativeEncoder sparkMaxEncoder = sparkMax.getEncoder();
  private DigitalInput limitSwitch = new DigitalInput(TURRET_LIMIT_SWITCH_PORT);

  private PIDController turretPID = new PIDController(TURRET_P, 0, TURRET_D);

  @Log
  private double omega = 0;
  private double lastTotalVelocity = 0;

  private SimEncoder turretSimEncoder = new SimEncoder();
  private SlewRateLimiter voltageLimiter = new SlewRateLimiter(0.1);
  // Open-loop drive in turret radians per second
  private SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(

    TURRET_FF[0],
    TURRET_FF[1],
    TURRET_FF[2]);
  private LinearSystemSim<N2, N1, N1> turretSim = new LinearSystemSim<N2, N1, N1>(
    LinearSystemId.identifyPositionSystem(TURRET_FF[1], TURRET_FF[2])
    );

  /** Creates a new TurretS. */
  public TurretS() {
    sparkMax.restoreFactoryDefaults();
    // Automatically multiply NEO rotations to read encoder in turret radians.
    sparkMaxEncoder.setPositionConversionFactor(2 * Math.PI / NEO_REVOLUTIONS_PER_TURRET_REVOLUTION);
    sparkMaxEncoder.setVelocityConversionFactor(2 * Math.PI / NEO_REVOLUTIONS_PER_TURRET_REVOLUTION / 60);
    sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) SOFT_LIMIT_FORWARD_RADIAN);
    sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) SOFT_LIMIT_REVERSE_RADIAN);
    turretPID.setTolerance(TURRET_PID_ERROR, 0);
    sparkMax.setSmartCurrentLimit(20, 20, 0);
    turretPID.setIntegratorRange(0, 0);
    SmartDashboard.putBoolean("requestTurrReset", false);

    if(RobotBase.isReal()) {
      sparkMaxEncoder.setPosition(Math.PI);
    }
    else {
      turretSimEncoder.setPosition(Math.PI);
    }
    getEncoderCounts();
  }

  /**
   * Sets the speed of the turret to 0 if the joystick is less than 1.5% away from
   * 0
   * 
   * @param value The speed of the turret
   * @return The speed of the turret
   */
  public double deadbandJoysticks(double value) {
    if (Math.abs(value) < TURRET_DEADBAND) {
      value = 0;
    }
    return value;
  }

  /**
   * Gets encoder counts from spark max
   * 
   * @return the encoder counts
   */
  @Log(name = "turretPosition")
  public double getEncoderCounts() {
    if(RobotBase.isReal()){
      return sparkMaxEncoder.getPosition();
    }
    else {
      return turretSimEncoder.getPosition();
    }
  }

  public Rotation2d getRobotToTurretRotation() {
    return new Rotation2d(getEncoderCounts());
  }
  
  /**
   * Sets the turn speed of the turret
   * 
   * @param speed The turn speed of the turret
   */
  public void turnSpeed(double speed) {
    sparkMax.setVoltage(turretFF.calculate(speed * TURRET_MAX_SPEED));

  }

  /**
   * Turns the turret towards the homing switch at a safe speed.
   */
   public void turnHoming() {
     turnSpeed(TURRET_HOMING_SPEED);
   }

  /**
   * Stops the motor.
   */
  public void stopMotor() {
    sparkMax.setVoltage(0);
  }

  /**
   * Resets encoder count.
   */
  public void resetEncoder() {
    resetEncoder(Math.PI);
  }

    /**
   * Resets encoder count.
   */
  public void resetEncoder(double radians) {
    if(RobotBase.isReal()) {
      sparkMaxEncoder.setPosition(radians);
    }
    else {
      turretSimEncoder.setPosition(radians);
    }
  }

  /**
   * Determines whether the turret is homed.
   * 
   * @return True if homing switch is triggered.
   */
  @Log(name = "turretHomed")
  public boolean getIsHomed() {
    return !limitSwitch.get();
  }

  public void setTransformVelocity(double omega) {
    this.omega = omega;
  }

  /**
   * Gets the velocity of the NEO motor.
   * 
   * @return The velocity of the motor, in turret radians per second
   */
  @Log(name = "turretVelocity")
  public double getVelocity() {
    if(RobotBase.isReal()) {
      return sparkMaxEncoder.getVelocity();
    }
    else {
      return turretSimEncoder.getVelocity();
    }
    
  }

  /**
   * sets the speed of the turret from -1 to 1, ensures the position of the turret
   * is
   * within the soft limit, calculates the power to give to the motor
   * 
   * @param target The desired angle, relative to the +x axis of the robot coordinate frame.
   */
  public void setTurretAngle(Rotation2d target) {
    // the given target switches from -pi to pi as it crosses the back of the robot, so we mod by 2pi
    double targetPosition = target.getRadians();
    if(targetPosition < 0) {
      targetPosition = 2*Math.PI + targetPosition;
    }

    SmartDashboard.putNumber("targetUnadj", targetPosition);
    targetPosition = MathUtil.clamp(targetPosition, SOFT_LIMIT_REVERSE_RADIAN, SOFT_LIMIT_FORWARD_RADIAN);
    // now the target wraps at 0 or 2pi, giving a nice continuous range over the places the turret can actually be.
    double currentPosition = getEncoderCounts();//Math.IEEEremainder(getEncoderCounts(), (2 * Math.PI));

    SmartDashboard.putNumber("turretPos", currentPosition);
    SmartDashboard.putNumber("turretTgt", targetPosition);
    double pidVelocity = turretPID.calculate(currentPosition, targetPosition); //radians per sec
    double totalVelocity = pidVelocity + omega;
    double acceleration = (totalVelocity - lastTotalVelocity) / 0.02;
    lastTotalVelocity = totalVelocity;
    double voltage =
    MathUtil.clamp(
      turretFF.calculate(totalVelocity, acceleration), -1.5, 1.5);
    
    sparkMax.setVoltage(voltageLimiter.calculate(voltage));
  }

  public void simulationPeriodic() {
    double voltage = sparkMax.getAppliedOutput();

    if(DriverStation.isEnabled()) {
        turretSim.setInput(voltage - (TURRET_FF[0]*Math.signum(voltage)));
    }
    else {
      turretSim.setInput(0);
    }

    turretSim.update(0.02);
    double newPosition = turretSim.getOutput().get(0, 0);
    SmartDashboard.putNumber("turretSimPos", newPosition);
    turretSimEncoder.setPosition(newPosition);
  }
  /**
   * gets the position error
   * 
   * @return The position error
   */
  @Log
  public double getError() {
    return turretPID.getPositionError();
  }

  /**
   * determines whether the turret is at the target angle
   * 
   * @return True if the turret is at the target angle
   */
  @Log
  public boolean isAtTarget() {
    return turretPID.atSetpoint();
  }

  @Override
  public void periodic() {
    if(SmartDashboard.getBoolean("requestTurrReset", false)){
      resetEncoder(Math.PI);
      SmartDashboard.putBoolean("requestTurrReset", false);
    }
  }

}