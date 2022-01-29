// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterS extends SubsystemBase {
  private final CANSparkMax frontSparkMax = new CANSparkMax(40, MotorType.kBrushless);
  private final CANSparkMax backSparkMax = new CANSparkMax(41, MotorType.kBrushless);
  private RelativeEncoder frontEncoder;
  private RelativeEncoder backEncoder;

  /** Creates a new ShooterS. */
  public ShooterS() {
    frontSparkMax.restoreFactoryDefaults();
    backSparkMax.restoreFactoryDefaults();

    frontEncoder = frontSparkMax.getEncoder();
    backEncoder = backSparkMax.getEncoder();
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
    frontSparkMax.set(speed);
  }

  /**
   * Sets the speed of the back motor
   * 
   * @param speed Speed value for the back motor
   */
  public void setBackSpeed(double speed) {
    backSparkMax.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
