// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class LightS {

  private static LightS m_instance = new LightS();
  private Spark spark = new Spark(Constants.PWM_PORT_LED);

  /** Creates a new LedS. */
  private LightS() {
  }

  public static LightS getInstance() {
      return m_instance;
  }

  private TreeSet<States> m_states = new TreeSet<>();


  /**
   * Different states of the robot, with an integer that determines the priority
   * of the state (the lower the number, the higher the priority)
   */
  public static enum States {
    Disabled(Constants.LED_SOLID_GREEN), // set in robotPeriodic
    Error(Constants.LED_PATTERN_RED),
    Climbing(Constants.LED_PARTY_MODE), // set through triggers in RobotContainer
    ShooterAndDistanceReady(Constants.LED_GOLD_SOLID), // ditto
    ShooterReady(Constants.LED_GREEN_RAINBOW), // ditto
    Shooting(Constants.LED_PATTERN_GREEN), //ditto
    Intaking(Constants.LED_PATTERN_GREEN), // set from MainCommandFactory.createIntakeCG`
    Default(Constants.LED_SOLID_GREEN);

    public final double lightCode;

    private States(double lightCode) {
      this.lightCode = lightCode;
    }
  }

  // currentStates = {Disabled, Climbing, EjectingWrongColor, Intaking, Shooting,
  // Default}

  /**
   * Requests the current state of the robot, determines whether the requested
   * state is a higher priority than the current state, sets the current state to
   * the requested state
   * 
   * @param state The requested state of the robot when the method is called
   */
  public void requestState(States state) {
    m_states.add(state);
  }

  /**
   * Periodically checks the current state of the robot and sets the LEDs to the
   * corresponding light pattern
   */
  public void periodic() {
    requestState(States.Default);
    spark.set(m_states.first().lightCode);
    m_states.removeAll(Set.of(States.values()));
  }
}
