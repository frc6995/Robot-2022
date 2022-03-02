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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberS extends SubsystemBase {
  private CANSparkMax sparkMax = new CANSparkMax(Constants.CAN_ID_CLIMBER_MOTOR, MotorType.kBrushless);
  private RelativeEncoder sparkMaxEncoder = sparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private PIDController climberPID = new PIDController(Constants.CLIMBER_P_CONSTANT, Constants.CLIMBER_I_CONSTANT, Constants.CLIMBER_D_CONSTANT);
  private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.DOUBLE_SOLENOID_CLIMBER_UP, 
  Constants.DOUBLE_SOLENOID_CLIMBER_DOWN);
  private CANSparkMax backSparkMax = new CANSparkMax(Constants.CAN_ID_BACK_CLIMB_MOTOR, MotorType.kBrushless);
  private RelativeEncoder sparkMaxEncoderTwo = backSparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private PIDController climberPIDTwo = new PIDController(Constants.BACK_CLIMBER_P_CONSTANT, Constants.BACK_CLIMBER_I_CONSTANT, 
  Constants.BACK_CLIMBER_D_CONSTANT);
  /** Creates a new ClimberS. */
  public ClimberS() {
    sparkMax.restoreFactoryDefaults();
    sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    sparkMax.setSoftLimit(SoftLimitDirection.kForward, Constants.CLIMBER_SOFT_LIMIT_FORWARD);
    sparkMax.setSoftLimit(SoftLimitDirection.kReverse, Constants.CLIMBER_SOFT_LIMIT_BACK);
    climberPID.setTolerance(Constants.CLIMBER_PID_ERROR, 0);
    climberPID.setIntegratorRange(0, 0);
    climberPIDTwo.setTolerance(Constants.CLIMBER_PID_ERROR, 0);
    climberPIDTwo.setIntegratorRange(0, 0);
  }

  /**
   * Extends the climber at a speed of 0.1
   */
  public void extend() {
    sparkMax.setVoltage(0.1 * 12);
  }

  /**
   * Retracts the climber at a speed of -0.1
   */
  public void retract() {
    sparkMax.setVoltage(-0.1 * 12);
  }

  /**
   * Gets the number of encoder counts 
   * 
   * @return the position of the climber
   */
  public double getEncoderCounts() {
    return sparkMaxEncoder.getPosition();
  }

  /**
   * Stops the motor
   */
  public void stopMotor() {
    sparkMax.setVoltage(0);
  }

  /**
   * sets the speed of the climber from -1 to 1
   * ensures the position of the climber is within the soft limit
   * calculates the power to give to the motor
   * 
   * @param target the desired amount of rotations
   */
  public void setClimberRotations(double target) {
    if (target > Constants.CLIMBER_SOFT_LIMIT_FORWARD) {
      target = Constants.CLIMBER_SOFT_LIMIT_FORWARD;
    }
    if (target < Constants.CLIMBER_SOFT_LIMIT_BACK) {
      target = Constants.CLIMBER_SOFT_LIMIT_BACK;
    }
    sparkMax.setVoltage(MathUtil.clamp(climberPID.calculate(this.getEncoderCounts(), target), -1, 1) * 12);
  }

  /**
   * gets the position error
   * 
   * @return the position error
   */
  public double error() {
    return climberPID.getPositionError();
  }

  /**
   * Determines whether the climber is at the desired position
   * 
   * @return true or false if the climber is at the desired position
   */
  public boolean atTarget(){
    return climberPID.atSetpoint();
  }

  /**
   * Extends the climber so that it folds up
   */
  public void climberUp() {
    doubleSolenoid.set(Value.kForward);
  }

  /**
   * Retracts the climber so that it folds down
   */
  public void climberDown() {
    doubleSolenoid.set(Value.kReverse);
  }

  /**
   * Sets the climber to Off so that it locks in place
   */
  public void climberOff() {
    doubleSolenoid.set(Value.kOff);
  }

  /**
   * Sets back spark max speed to 0.1.
   */
  public void extendArmTwo() {
    backSparkMax.setVoltage(0.1 * 12);
  }
  /**
   * Set back spark max speed to -0.1.
   */
  public void retractArmTwo() {
    backSparkMax.setVoltage(-0.1 * 12);
  }
  /**
   * Get encoder counts for 2nd spark max.
   * 
   * @return sparkMaxEncoderTwo.getPosition()
   */
  public double getEncoderCountsArmTwo() {
    return sparkMaxEncoderTwo.getPosition();
  }

  /**
   * Stops the motor
   */
  public void stopMotorArmTwo() {
    backSparkMax.setVoltage(0);
  }
  /**
   * Set rotations for motor.
   * 
   * @param target
   */
  public void setClimberRotationsArmTwo(double target) {
    if (target > Constants.CLIMBER_SOFT_LIMIT_FORWARD) {
      target = Constants.CLIMBER_SOFT_LIMIT_FORWARD;
    }
    if (target < Constants.CLIMBER_SOFT_LIMIT_BACK) {
      target = Constants.CLIMBER_SOFT_LIMIT_BACK;
    }
    backSparkMax.setVoltage(MathUtil.clamp(climberPIDTwo.calculate(this.getEncoderCountsArmTwo(), target), -1, 1) * 12);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
