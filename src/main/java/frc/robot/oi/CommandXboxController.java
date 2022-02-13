
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Provides Triggers for binding commands to an XboxController's buttons. Additionally offers
 * getters for retrieving axis values.
 */
public class CommandXboxController extends GenericHID {
  // reuses the Button and Axis enums from the original XboxController

  private Trigger m_leftBumper;
  private Trigger m_rightBumper;
  private Trigger m_leftStick;
  private Trigger m_rightStick;
  private Trigger m_a;
  private Trigger m_b;
  private Trigger m_x;
  private Trigger m_y;
  private Trigger m_backButton;
  private Trigger m_startButton;

  @SuppressWarnings("checkstyle:MemberName")
  public final CommandControllerPOV pov;

  /**
   * Constructs a CommandXboxController.
   *
   * @param port The Driver Station port to initialize it on.
   */
  public CommandXboxController(final int port) {
    super(port);
    pov = new CommandControllerPOV(this);
  }

  /**
   * Returns the left bumper's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger leftBumper() {
    if (m_leftBumper == null) {
      m_leftBumper = new Trigger(()->this.getRawButton(XboxController.Button.kLeftBumper.value));
    }

    return m_leftBumper;
  }

  /**
   * Returns the right bumper's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger rightBumper() {
    if (m_rightBumper == null) {
      m_rightBumper = new Trigger(()->this.getRawButton(XboxController.Button.kRightBumper.value));
    }

    return m_rightBumper;
  }

  /**
   * Returns the left stick's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger leftStick() {
    if (m_leftStick == null) {
      m_leftStick = new Trigger(()->this.getRawButton(XboxController.Button.kLeftStick.value));
    }

    return m_leftStick;
  }

  /**
   * Returns the right stick's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger rightStick() {
    if (m_rightStick == null) {
      m_rightStick = new Trigger(()->this.getRawButton(XboxController.Button.kRightStick.value));
    }

    return m_rightStick;
  }

  /**
   * Returns the A button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  @SuppressWarnings("checkstyle:MethodName")
  public Trigger a() {
    if (m_a == null) {
      m_a = new Trigger(()->this.getRawButton(XboxController.Button.kA.value));
    }

    return m_a;
  }

  /**
   * Returns the B button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  @SuppressWarnings("checkstyle:MethodName")
  public Trigger b() {
    if (m_b == null) {
      m_b = new Trigger(()->this.getRawButton(XboxController.Button.kB.value));
    }

    return m_b;
  }

  /**
   * Returns the X button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  @SuppressWarnings("checkstyle:MethodName")
  public Trigger x() {
    if (m_x == null) {
      m_x = new Trigger(()->this.getRawButton(XboxController.Button.kX.value));
    }

    return m_x;
  }

  /**
   * Returns the Y button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  @SuppressWarnings("checkstyle:MethodName")
  public Trigger y() {
    if (m_y == null) {
      m_y = new Trigger(()->this.getRawButton(XboxController.Button.kY.value));
    }

    return m_y;
  }

  /**
   * Returns the back button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger back() {
    if (m_backButton == null) {
      m_backButton = new Trigger(()->this.getRawButton(XboxController.Button.kBack.value));
    }

    return m_backButton;
  }

  /**
   * Returns the start button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger start() {
    if (m_startButton == null) {
      m_startButton = new Trigger(()->this.getRawButton(XboxController.Button.kStart.value));
    }

    return m_startButton;
  }

  /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return getRawAxis(XboxController.Axis.kLeftX.value);
  }

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightX() {
    return getRawAxis(XboxController.Axis.kRightX.value);
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return getRawAxis(XboxController.Axis.kLeftY.value);
  }

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightY() {
    return getRawAxis(XboxController.Axis.kRightY.value);
  }

  /**
   * Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getLeftTriggerAxis() {
    return getRawAxis(XboxController.Axis.kLeftTrigger.value);
  }

  /**
   * Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getRightTriggerAxis() {
    return getRawAxis(XboxController.Axis.kRightTrigger.value);
  }
}
