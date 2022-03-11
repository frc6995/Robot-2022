package frc.robot.subsystems;

import static frc.robot.Constants.CAN_ID_TURRET;
import static frc.robot.Constants.NEO_REVOLUTIONS_PER_TURRET_REVOLUTION;
import static frc.robot.Constants.SOFT_LIMIT_FORWARD_RADIAN;
import static frc.robot.Constants.SOFT_LIMIT_REVERSE_RADIAN;
import static frc.robot.Constants.TURRET_D;
import static frc.robot.Constants.TURRET_DEADBAND;
import static frc.robot.Constants.TURRET_FF;
import static frc.robot.Constants.TURRET_HOMING_SPEED;
import static frc.robot.Constants.TURRET_LIMIT_SWITCH_PORT;
import static frc.robot.Constants.TURRET_MAX_SPEED;
import static frc.robot.Constants.TURRET_P;
import static frc.robot.Constants.TURRET_PID_ERROR;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private ProfiledPIDController turretPID = new ProfiledPIDController(TURRET_P, 0, TURRET_D, new Constraints(Math.PI, 2));

  @Log
  private double omega = 0;

  private SimEncoder turretSimEncoder = new SimEncoder();
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
    sparkMax.setIdleMode(IdleMode.kBrake);
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
   * @param speed The turn speed of the turret [-1..1]
   */
  public void turnSpeedOpenLoop(double speed) {
    sparkMax.setVoltage(turretFF.calculate(speed * TURRET_MAX_SPEED));
  }

  /**
   * Set the velocity of the turret 
   */
  public void turnVelocityOpenLoop(double velocity) {
    //velocity = MathUtil.clamp(velocity, -TURRET_MAX_SPEED, TURRET_MAX_SPEED);
    setVelocity(velocity);
  }

  /**
   * Turns the turret towards the homing switch at a safe speed.
   */
   public void turnHoming() {
     turnSpeedOpenLoop(TURRET_HOMING_SPEED);
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
      System.out.println("turret reset to " + radians);
      turretSimEncoder.setPosition(radians);
      Matrix<N2, N1> newState = new Matrix<N2, N1>(Nat.N2(), Nat.N1());
      newState.set(0, 0, radians);
      newState.set(1, 0, 0);
      turretSim.setState(newState);
    }
  }

  public void setVelocity(double velocity) {
    sparkMax.setVoltage(
        /*velocityLimiter.calculate*/(turretFF.calculate(velocity))
    );
  }

  public void setVoltage(double voltage) {
    sparkMax.setVoltage(voltage);
  }

  public boolean isTargetInRange (Rotation2d target) {
    double targetRadians = modulus(target);

    if(targetRadians > Constants.SOFT_LIMIT_FORWARD_RADIAN) {
      return false;
    }
    else if(targetRadians < Constants.SOFT_LIMIT_REVERSE_RADIAN) {
      return false;
    }
    return true;
  }

  public double modulus(Rotation2d rotation) {
    double targetPosition = rotation.getRadians();
    if(targetPosition < 0) {
      targetPosition = 2*Math.PI + targetPosition;
    }
    return targetPosition;
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

  public void resetPID() {
    turretPID.reset(getEncoderCounts());
  }

  public void setTurretAngle(Rotation2d target) {
    setTurretAngle(target, 0);
  }
  /**
   * sets the speed of the turret from -1 to 1, ensures the position of the turret
   * is
   * within the soft limit, calculates the power to give to the motor
   * 
   * @param target The desired angle, relative to the +x axis of the robot coordinate frame.
   */
  public void setTurretAngle(Rotation2d target, double angleOffset) {
    // the given target switches from -pi to pi as it crosses the back of the robot, so we mod by 2pi
    double targetPosition = target.getRadians();
    if(targetPosition < 0) {
      targetPosition = 2*Math.PI + targetPosition;
    }
    // Clamp the angle offset so that at least one side can actually handle it.
    angleOffset = MathUtil.clamp(angleOffset, 0, SOFT_LIMIT_FORWARD_RADIAN - Math.PI);

    if(targetPosition + angleOffset > SOFT_LIMIT_FORWARD_RADIAN) {
        targetPosition -= angleOffset;
    } 
    else {
        targetPosition += angleOffset;
    }

    SmartDashboard.putNumber("targetUnadj", targetPosition);
      targetPosition = MathUtil.clamp(targetPosition, SOFT_LIMIT_REVERSE_RADIAN, SOFT_LIMIT_FORWARD_RADIAN);
      // now the target wraps at 0 or 2pi, giving a nice continuous range over the places the turret can actually be.
      double currentPosition = getEncoderCounts();//Math.IEEEremainder(getEncoderCounts(), (2 * Math.PI));

      SmartDashboard.putNumber("turretPos", currentPosition);
      SmartDashboard.putNumber("turretTgt", targetPosition);
      double pidVelocity = turretPID.calculate(currentPosition, targetPosition); //radians per sec
      setVelocity(pidVelocity);
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
    newPosition = newPosition + Math.PI;
    SmartDashboard.putNumber("turretSimPos", newPosition);
    newPosition = MathUtil.clamp(newPosition, SOFT_LIMIT_REVERSE_RADIAN, SOFT_LIMIT_FORWARD_RADIAN);

    turretSimEncoder.setPosition(newPosition);
  }

  @Log
  public double getError(Rotation2d target) {
    return modulus(getRobotToTurretRotation()) - modulus(target);
  }

  /**
   * determines whether the turret is at the target angle
   * 
   * @return True if the turret is at the target angle
   */
  @Log
  public boolean isAtTarget(Rotation2d target) {
    return Math.abs(getError(target)) < Constants.TURRET_PID_ERROR;
  }
  
  @Override
  public void periodic() {
    if(DriverStation.isDisabled()) {
      stopMotor();
    }
    if(SmartDashboard.getBoolean("requestTurrReset", false)){
      resetEncoder(Math.PI);
      SmartDashboard.putBoolean("requestTurrReset", false);
    }
  }

}