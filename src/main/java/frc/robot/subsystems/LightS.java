// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightS extends SubsystemBase {

  public static states currentState;
  private Spark spark;

  /** Creates a new LedS. */
  public LightS() {
  }

  /**
   * Different states of the robot, with an integer that determines the priority
   * of the state (the lower the number, the higher the priority)
   */
  public enum states {
    Disabled(0),
    Climbing(1),
    IntakingWrongColor(2),
    Intaking(3),
    Shooting(4),
    Default(5);

    public int value;

    private states(int priority) {
      value = priority;
    }
  }

  /**
   * Requests the current state of the robot, determines whether the requested
   * state is a higher priority than the current state, sets the current state to
   * the requested state
   * 
   * @param state The requested state of the robot when the method is called
   */
  public void requestState(states state) {

    // if (currentState.value < stat.value) {}

    if (currentState.value >= state.value) {
      currentState = state;
    }
  }

  /**
   * Periodically checks the current state of the robot and sets the LEDs to the
   * corresponding light pattern
   */
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