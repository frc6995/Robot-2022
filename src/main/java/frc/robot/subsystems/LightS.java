// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightS extends SubsystemBase {

  public states currentState;
  private Spark spark;

  /** Creates a new LedS. */
  public LightS() {
  }

  public enum states {
    Disabled,
    IntakingWrongColor,
    Intaking,
    Shooting,
    Climbing,
    Default;
  }

  public void requestState(states state) {
    currentState = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch (currentState) {
      case Disabled:
        spark.set(Constants.LED_SOLID_GREEN);
        break;
      case IntakingWrongColor:
        spark.set(Constants.LED_PATTERN_RED);
        break;
      case Intaking:
        spark.set(Constants.LED_PATTERN_GREEN);
        break;
      case Shooting:
        spark.set(Constants.LED_PATTERN_GREEN);
        break;
      case Climbing:
        spark.set(Constants.LED_PARTY_MODE);
        break;
      case Default:
        spark.set(Constants.LED_SOLID_GREEN);
        break;
    }
  }
}