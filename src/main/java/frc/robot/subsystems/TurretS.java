// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretS extends SubsystemBase {
  private CANSparkMax sparkMax = new CANSparkMax(Constants.CAN_ID_TURRET, MotorType.kBrushless);
  private RelativeEncoder sparkMaxEncoder = sparkMax.getEncoder();
  /** Creates a new TurretS. */
  public TurretS() {
    sparkMax.restoreFactoryDefaults();
  }

  public double getEncoderCounts(){
    return sparkMaxEncoder.getPosition();
  }

  public double degreesToEncoderCounts(double degrees){
    double encoderCounts = degrees * Constants.ENCODER_COUNTS_PER_TURRET_DEGREE;
    return encoderCounts;
  }

  public double encoderCountsToDegrees(double encoderCounts){
    double degrees = encoderCounts / Constants.ENCODER_COUNTS_PER_TURRET_DEGREE;
    return degrees;
  }

  public void turnMaxSpeed(){
    sparkMax.set(Constants.TURRET_SPEED);
  }
  public void stopMotor(){
    sparkMax.set(0);
  }

  public void resetEncoder() {
    sparkMaxEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turret degrees", encoderCountsToDegrees(getEncoderCounts()));
    SmartDashboard.putNumber("turret counts", getEncoderCounts());
  }
}