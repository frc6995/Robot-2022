// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The turret subsystem. It contains methods to get encoder counts and converts
 * them to degrees.
 * 
 * @author Benjamin Su
 * @author Noah Kim
 */
public class TurretS extends SubsystemBase implements Loggable {
  private CANSparkMax sparkMax = new CANSparkMax(Constants.CAN_ID_TURRET, MotorType.kBrushless);
  private RelativeEncoder sparkMaxEncoder = sparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT_NUMBER);

  /** Creates a new TurretS. */
  public TurretS() {
    sparkMax.restoreFactoryDefaults();
    //Automatically multiply NEO rotations to read encoder in turret degrees.
    sparkMaxEncoder.setPositionConversionFactor(360 / Constants.NEO_REVOLUTIONS_PER_TURRET_REVOLUTION);
    sparkMaxEncoder.setVelocityConversionFactor(360 / Constants.NEO_REVOLUTIONS_PER_TURRET_REVOLUTION / 60);
    sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    sparkMax.setSoftLimit(SoftLimitDirection.kForward, Constants.SOFT_LIMIT_FORWARD_DEGREE);
    sparkMax.setSoftLimit(SoftLimitDirection.kReverse, Constants.SOFT_LIMIT_REVERSE_DEGREE);
  }

  /**
   *Gets encoder counts from spark max
   * 
   * @return the encoder counts
   */
  public double getEncoderCounts(){
    return sparkMaxEncoder.getPosition();
  }

  /**
   * Converts degrees to encoder counts
   * 
   * @param degrees degrees coming from limelight
   * @return returns degrees in encoder counts
   */
  public double degreesToEncoderCounts(double degrees){
    double encoderCounts = degrees * Constants.ENCODER_COUNTS_PER_TURRET_DEGREE;
    return encoderCounts;
  }

  /**
   * Converts encoder counts to degrees
   * 
   * @param encoderCounts encoder counts from spark max
   * @return returns encoder counts in degrees
   */
  public double encoderCountsToDegrees(double encoderCounts){
    double degrees = encoderCounts / Constants.ENCODER_COUNTS_PER_TURRET_DEGREE;
    return degrees;
  }

  /**
   * Max speed of turret
   */
  public void turnMaxSpeed(){
    sparkMax.set(Constants.TURRET_SPEED);
  }

  /**
   * Stops the motor
   */
  public void stopMotor(){
    sparkMax.set(0);
  }

  /**
   * Resets encoder count 
   */
  public void resetEncoder() {
    sparkMaxEncoder.setPosition(0);
  }

  public boolean getIsHomed() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turret degrees", /*encoderCountsToDegrees*/(getEncoderCounts()));
    SmartDashboard.putNumber("turret counts", getEncoderCounts());
    SmartDashboard.putBoolean("limit switch true or false", getIsHomed());
  }
}