package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private CANSparkMax sparkMax = new CANSparkMax(Constants.CAN_ID_TURRET, MotorType.kBrushless);
  private RelativeEncoder sparkMaxEncoder = sparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private DigitalInput limitSwitch = new DigitalInput(Constants.TURRET_LIMIT_SWITCH_PORT);
  private PIDController turretPID = new PIDController(Constants.TURRET_P, 0, 0);

  @Log
  private double omega = 0;

  private SimEncoder turretSimEncoder = new SimEncoder();
  // Open-loop drive in turret radians per second
  private SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(

    Constants.TURRET_FF[0],
    Constants.TURRET_FF[1],
    Constants.TURRET_FF[2]);
  private LinearSystemSim<N2, N1, N1> turretSim = new LinearSystemSim<N2, N1, N1>(
    LinearSystemId.identifyPositionSystem(Constants.TURRET_FF[1], Constants.TURRET_FF[2])
    );
  private boolean targetInRange;
  private Trigger velocityThresholdTrigger = new Trigger(()->{return Math.abs(omega) < Math.PI;}).debounce(0.5, DebounceType.kFalling);
  private Trigger targetInRangeTrigger = new Trigger(()->targetInRange).debounce(0.25);
  private Trigger safeToMoveTrigger = velocityThresholdTrigger.and(targetInRangeTrigger);

  /** Creates a new TurretS. */
  public TurretS() {
    sparkMax.restoreFactoryDefaults();
    // Automatically multiply NEO rotations to read encoder in turret radians.
    sparkMaxEncoder.setPositionConversionFactor(2 * Math.PI / Constants.NEO_REVOLUTIONS_PER_TURRET_REVOLUTION);
    sparkMaxEncoder.setVelocityConversionFactor(2 * Math.PI / Constants.NEO_REVOLUTIONS_PER_TURRET_REVOLUTION / 60);
    sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) Units.radiansToDegrees(Constants.SOFT_LIMIT_FORWARD_RADIAN));
    sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.radiansToDegrees(Constants.SOFT_LIMIT_REVERSE_RADIAN));
    turretPID.setTolerance(Constants.TURRET_PID_ERROR, 0);
    sparkMax.setSmartCurrentLimit(20, 20, 0);
    turretPID.setIntegratorRange(0, 0);
  }

  /**
   * Sets the speed of the turret to 0 if the joystick is less than 1.5% away from
   * 0
   * 
   * @param value The speed of the turret
   * @return The speed of the turret
   */
  public double deadbandJoysticks(double value) {
    if (Math.abs(value) < Constants.TURRET_DEADBAND) {
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

  public Rotation2d getRotation2d() {
    return new Rotation2d(getEncoderCounts());
  }
  
  /**
   * Sets the turn speed of the turret
   * 
   * @param speed The turn speed of the turret
   */
  public void turnSpeed(double speed) {
    sparkMax.setVoltage(turretFF.calculate(speed * Constants.TURRET_MAX_SPEED));

  }

  /**
   * Turns the turret towards the homing switch at a safe speed.
   */
   public void turnHoming() {
     turnSpeed(Constants.TURRET_HOMING_SPEED);
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
    resetEncoder(0);
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
    Rotation2d targetZeroRelative = target.rotateBy(Constants.ROBOT_TO_TURRET_ZERO_ROT.unaryMinus());
    if (targetZeroRelative.getRadians() >= Constants.SOFT_LIMIT_FORWARD_RADIAN) {
      targetInRange = false;
    }
    else if (targetZeroRelative.getRadians() <= Constants.SOFT_LIMIT_REVERSE_RADIAN) {
      targetInRange = false;
    } else {
      targetInRange = true;
    }
    double pidVelocity = turretPID.calculate(getRotation2d().getRadians(), targetZeroRelative.getRadians()); //radians per sec
    SmartDashboard.putNumber("turretVelo", pidVelocity);
    double voltage =
      turretFF.calculate(pidVelocity + omega    // MathUtil.clamp(
        //   velocity,
        //   -Constants.TURRET_MAX_SPEED,
        //   Constants.TURRET_MAX_SPEED
        // )  
      );
    if(safeToMoveTrigger.get()) {//If the target has been in range for more than 1.5 seconds, 
      sparkMax.setVoltage(voltage);
    }
    else{
      sparkMax.setVoltage(0);
    }
  }

  public void simulationPeriodic() {
    if(DriverStation.isEnabled()) {
      turretSim.setInput(sparkMax.getAppliedOutput() - (Constants.TURRET_FF[0]*Math.signum(sparkMax.getAppliedOutput())));
    }
    else {
      turretSim.setInput(0);
    }

    turretSim.update(0.02);
    turretSimEncoder.setPosition(MathUtil.clamp(turretSim.getOutput().get(0, 0), Constants.SOFT_LIMIT_REVERSE_RADIAN, Constants.SOFT_LIMIT_FORWARD_RADIAN));
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
  public boolean isAtTarget() {
    return turretPID.atSetpoint();
  }

  @Override
  public void periodic() {
  }

}