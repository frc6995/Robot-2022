// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The turret subsystem. It contains methods to get encoder counts and converts
 * them to degrees. It uses a PID controller to control the turret position.
 * 
 * @author Benjamin Su
 * @author Noah Kim
 */
public class TurretS extends SubsystemBase implements Loggable {
  private CANSparkMax sparkMax = new CANSparkMax(Constants.CAN_ID_TURRET, MotorType.kBrushless);
  private RelativeEncoder sparkMaxEncoder = sparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private DigitalInput limitSwitch = new DigitalInput(Constants.TURRET_LIMIT_SWITCH_PORT);
  private PIDController turretPID = new PIDController(Constants.TURRET_P, 0, 0);
  // Open-loop drive in turret rotations per second
  private SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(
    Constants.TURRET_FF[0],
    Constants.TURRET_FF[1],
    Constants.TURRET_FF[2]);

  /** Creates a new TurretS. */
  public TurretS() {
    sparkMax.restoreFactoryDefaults();
    // Automatically multiply NEO rotations to read encoder in turret degrees.
    sparkMaxEncoder.setPositionConversionFactor(360 / Constants.NEO_REVOLUTIONS_PER_TURRET_REVOLUTION);
    sparkMaxEncoder.setVelocityConversionFactor(360 / Constants.NEO_REVOLUTIONS_PER_TURRET_REVOLUTION / 60);
    sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    sparkMax.setSoftLimit(SoftLimitDirection.kForward, Constants.SOFT_LIMIT_FORWARD_DEGREE);
    sparkMax.setSoftLimit(SoftLimitDirection.kReverse, Constants.SOFT_LIMIT_REVERSE_DEGREE);
    turretPID.setTolerance(Constants.TURRET_PID_ERROR, 0);
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
  @Log(name="turretPosition")
  public double getEncoderCounts() {
    return sparkMaxEncoder.getPosition();
  }

  /**
   * Sets the turn speed of the turret
   * 
   * @param speed The turn speed of the turret
   */
  public void turnSpeed(double speed) {
    speed = deadbandJoysticks(speed);
    sparkMax.set(speed);
  }

  /**
   * Turns the turret towards the homing switch at a safe speed.
   */
   public void turnHoming() {
     sparkMax.set(Constants.TURRET_HOMING_SPEED);
   }

  /**
   * Stops the motor.
   */
  public void stopMotor() {
    sparkMax.set(0);
  }

  /**
   * Resets encoder count.
   */
  public void resetEncoder() {
    sparkMaxEncoder.setPosition(0);
  }

  /**
   * Determines whether the turret is homed.
   * 
   * @return True if homing switch is triggered.
   */
  @Log(name="turretHomed")
  public boolean getIsHomed() {
    return !limitSwitch.get();
  }

  /**
   * Gets the velocity of the NEO motor.
   * 
   * @return The velocity of the motor, in turret rotations per second
   */
  @Log(name="turretVelocity")
  public double getVelocity() {
    return sparkMaxEncoder.getVelocity();
  }

  /**
   * sets the speed of the turret from -1 to 1, ensures the position of the turret is 
   * within the soft limit, calculates the power to give to the motor
   * 
   * @param target The desired angle
   */
  public void setTurretAngle(double target) {
    if (target > Constants.SOFT_LIMIT_FORWARD_DEGREE) {
      target = Constants.SOFT_LIMIT_FORWARD_DEGREE;
    }
    if (target < Constants.SOFT_LIMIT_REVERSE_DEGREE) {
      target = Constants.SOFT_LIMIT_REVERSE_DEGREE;
    }
    sparkMax.setVoltage(
      MathUtil.clamp(
        turretFF.calculate(
          turretPID.calculate(this.getEncoderCounts(), target)
        ),
         -3, 3)
    );
    
  }

  /**
   * gets the position error
   * 
   * @return The position error
   */
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
  public void periodic() {}

}