// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterS extends SubsystemBase {
  private final CANSparkMax frontSparkMax = new CANSparkMax(40, MotorType.kBrushless);
  private final CANSparkMax backSparkMax = new CANSparkMax(41, MotorType.kBrushless);
  private RelativeEncoder frontEncoder;
  private RelativeEncoder backEncoder;
  private PIDController frontPID = new PIDController(Constants.PROPORTIONAL_CONSTANT, 0, 0);
  private PIDController backPID = new PIDController(Constants.PROPORTIONAL_CONSTANT, 0, 0);

  /** Creates a new ShooterS. */
  public ShooterS() {
    frontSparkMax.restoreFactoryDefaults();
    backSparkMax.restoreFactoryDefaults();

    frontEncoder = frontSparkMax.getEncoder();
    backEncoder = backSparkMax.getEncoder();

    frontPID.setTolerance(Constants.SHOOTER_PID_ERROR, 0);
    backPID.setTolerance(Constants.SHOOTER_PID_ERROR, 0);


    frontPID.setIntegratorRange(0, 0);
    backPID.setIntegratorRange(0, 0);
  }

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
  public double getFrontEncoderSpeed() {
    return frontEncoder.getVelocity();
  }

  /**
   * Gets the back encoder speed
   * 
   * @return The velocity of the back motor
   */
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
    frontSparkMax.set(speed);
  }

  /**
   * Sets the speed of the back motor
   * 
   * @param speed Speed value for the back motor
   */
  public void setBackSpeed(double speed) {
    speed = deadbandJoystick(speed);
    backSparkMax.set(speed);
  }

  /**
   * sets the speed of the front motor using PID from -1 to 1
   * 
   * @param frontTargetRPM The target RPM of the front motor
   */
  public void pidFrontSpeed(double frontTargetRPM) {
    frontSparkMax.set(MathUtil.clamp(frontPID.calculate(getFrontEncoderSpeed(), frontTargetRPM), -1.0, 1.0));
        
  }

  /**
   * sets the speed of the back motor using PID from -1 to 1
   * 
   * @param backTargetRPM The target RPM of the front motor
   */
  public void pidBackSpeed(double backTargetRPM) {
    backSparkMax.set(MathUtil.clamp(backPID.calculate(getBackEncoderSpeed(), backTargetRPM), -1.0, 1.0));
  }

  /**
   * gets the velocity error of the front motor
   * 
   * @return The velocity error of the front motor
   */
  public double frontError() {
    return frontPID.getVelocityError();
  }

  /**
   * gets the velocity error of teh back motor
   * 
   * @return The velocity error of teh back motor
   */
  public double backError() {
    return backPID.getVelocityError();
  }

  /**
   * determines whether the front motor is at the target RPM
   * 
   * @return True if the front motor is at the target RPM
   */
  public boolean frontAtTarget() {
    return frontPID.atSetpoint();
  }

  /**
   * determines whether the back motor as at the target RPM
   * 
   * @return True if the back motor is at the target RPM
   */
  public boolean backAtTarget() {
    return backPID.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
