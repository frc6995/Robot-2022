// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Takes balls from the intake and holds it and sends it to the shooter.
 * 
 * @authors Jonas An and Ben Su
 */
public class MidtakeS extends SubsystemBase {
  /** Creates a new IntakeS. */
  private CANSparkMax frontSparkMax = new CANSparkMax(Constants.CAN_ID_MIDTAKE_FRONT,
      MotorType.kBrushless);
  private CANSparkMax backSparkMax = new CANSparkMax(Constants.CAN_ID_MIDTAKE_BACK,
      MotorType.kBrushless);
  private DigitalInput beamBreakTop = new DigitalInput(Constants.BEAM_BREAK_TOP_PORT_NUMBER);
  private DigitalInput beamBreakBottom = new DigitalInput(Constants.BEAM_BREAK_BOTTOM_PORT_NUMBER);
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  public MidtakeS() {
    frontSparkMax.restoreFactoryDefaults();
    frontSparkMax.setInverted(true);
    backSparkMax.restoreFactoryDefaults();
  }

  /**
   * Sets the speed of the front motor
   * 
   * @param frontSpeed the front speed
   */
  public void setFrontSpeed(double frontSpeed) {
    frontSparkMax.set(frontSpeed);
  }

  /**
   * Sets the speed of the back motor
   * 
   * @param backSpeed the back speed
   */
  public void setBackSpeed(double backSpeed) {
    backSparkMax.set(backSpeed);
  }

  /**
   * Spins both midtake motors at set speed.
   */
  public void spin() {
    spin(Constants.MIDTAKE_FRONT_MOTOR_SPEED);
  }

  /**
   * Spins both midtake motors at set speed.
   */
  public void spin(double speed) {
    setFrontSpeed(speed);
    setBackSpeed(speed);
  }

  /**
   * Returns whether the top Beam Break sensor is triggered
   *
   */
  public boolean getIsTopBeamBroken() {
    return beamBreakTop.get();
  }

  /**
   * Returns whether the bottom Beam Break sensor is triggered
   * 
   */
  public boolean getIsBottomBeamBroken() {
    return beamBreakBottom.get();
  }

  /**
   * Returns whether the color of the ball is the same as your team color
   * 
   * @return
   */
  public boolean getIsBallColorCorrect() {
    Color detectedColor = colorSensor.getColor();
    boolean isBallRed = detectedColor.red > detectedColor.blue;
    boolean areWeRed = DriverStation.getAlliance() == Alliance.Red;
    return isBallRed == areWeRed;
  }

  /**
   * Returns whether the color sensor's proximity sensor detects a ball.
   */
  public boolean getColorSensorDetectsBall() {
    return colorSensor.getProximity() > Constants.COLOR_SENSOR_PROXIMITY_THRESHOLD;
  }

  /**
   * Stops the midtake.
   */

  public void stop() {
    spin(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}