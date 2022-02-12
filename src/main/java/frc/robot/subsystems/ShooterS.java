// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterS extends SubsystemBase {
  private final CANSparkMax frontSparkMax = new CANSparkMax(Constants.CAN_ID_FRONT_SHOOTER_MOTOR, MotorType.kBrushless);
  private final CANSparkMax backSparkMax = new CANSparkMax(Constants.CAN_ID_BACK_SHOOTER_MOTOR, MotorType.kBrushless);
  private RelativeEncoder frontEncoder;
  private RelativeEncoder backEncoder;
  private PIDController frontPID = new PIDController(Constants.SHOOTER_FRONT_P, 0, 0);
  private PIDController backPID = new PIDController(Constants.SHOOTER_BACK_P, 0, 0);
  private SimpleMotorFeedforward frontFF = new SimpleMotorFeedforward(
    Constants.SHOOTER_FRONT_FF[0],
    Constants.SHOOTER_FRONT_FF[1],
    Constants.SHOOTER_FRONT_FF[2]);
  private SimpleMotorFeedforward backFF = new SimpleMotorFeedforward(
    Constants.SHOOTER_BACK_FF[0],
    Constants.SHOOTER_BACK_FF[1],
    Constants.SHOOTER_BACK_FF[2]);

  /** Creates a new ShooterS. */
  public ShooterS() {
    frontSparkMax.restoreFactoryDefaults();
    backSparkMax.restoreFactoryDefaults();
    frontSparkMax.setClosedLoopRampRate(6);
    backSparkMax.setClosedLoopRampRate(6);

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
    frontSparkMax.setVoltage(
        frontPID.calculate(getFrontEncoderSpeed()/60.0, frontTargetRPM/60.0) + frontFF.calculate(frontTargetRPM/60.0)
        );
        
  }

  /**
   * sets the speed of the back motor using PID from -1 to 1
   * 
   * @param backTargetRPM The target RPM of the front motor
   */
  public void pidBackSpeed(double backTargetRPM) {
    backSparkMax.setVoltage(
      backPID.calculate(getBackEncoderSpeed()/60.0, backTargetRPM/60.0) + backFF.calculate(backTargetRPM / 60.0)
    );
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

  public double getIndex(double distance) {
    int index = Math.max(Math.min(Constants.DISTANCES.length-2, getUnderId(distance, Constants.DISTANCES)), 0);
    double rpm = calcSpeed(index, index+1, distance, Constants.DISTANCES, Constants.SPEEDS);
    return rpm; 
  }

  public static int getUnderId(double distance, double[] DISTANCES_FEET) {
    int index = 0;
    for (int i = 0; i < DISTANCES_FEET.length; i++) {
        if (distance > DISTANCES_FEET[i]) {
            index = i;
        }
    }
    
    return index;
  }

  public static double calcSpeed(int smallerIndex, int biggerIndex, double distance, double[] DISTANCES_FEET, double[] RPMS) {
    double smallerRPM = RPMS[Math.max(smallerIndex, 0)];
    double biggerRPM = RPMS[Math.min(biggerIndex, RPMS.length-1)] + 0.0001; //add a tiny amount to avoid NaN if distance is out of range
    double smallerDistance = DISTANCES_FEET[Math.max(smallerIndex, 0)];
    double biggerDistance = DISTANCES_FEET[Math.min(biggerIndex, DISTANCES_FEET.length-1)] + 0.0001;
    double newRPM = ((biggerRPM - smallerRPM) / (biggerDistance - smallerDistance) * (distance - smallerDistance)) + smallerRPM;

    return newRPM;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Speed", getFrontEncoderSpeed());
    SmartDashboard.putNumber("Back Speed", getBackEncoderSpeed());
    // This method will be called once per scheduler run
  }
}
