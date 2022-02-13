// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Provides Triggers for binding commands to any GenericHID inherited class. */
public class CommandControllerPOV {
  private final GenericHID m_hid;
  private final int m_povNumber;

  private Trigger m_upButton; // 0 degrees
  private Trigger m_upRightButton; // 45 degrees
  private Trigger m_rightButton; // 90 degrees
  private Trigger m_downRightButton; // 135 degrees
  private Trigger m_downButton; // 180 degrees
  private Trigger m_downLeftButton; // 225 degrees
  private Trigger m_leftButton; // 270 degrees
  private Trigger m_upLeftButton; // 315 degrees
  private Trigger m_centerButton; // Center, which returns -1

  /**
   * Constructs a ControllerPOV.
   *
   * @param hid The HID controller to read the POV from.
   */
  public CommandControllerPOV(GenericHID hid) {
    this(hid, 0);
  }

  /**
   * Constructs a ControllerPOV.
   *
   * @param hid The HID controller to read the POV from.
   * @param povNumber The controller POV index to use.
   */
  public CommandControllerPOV(GenericHID hid, int povNumber) {
    m_hid = hid;
    m_povNumber = povNumber;
  }

  /**
   * Returns the upper (0 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger up() {
    if (m_upButton == null) {
      m_upButton = new Trigger(()->m_hid.getPOV(m_povNumber) == 0);
    }

    return m_upButton;
  }

  /**
   * Returns the upper-right (45 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger upRight() {
    if (m_upRightButton == null) {
      m_upRightButton = new Trigger(()->m_hid.getPOV(m_povNumber) == 45);
    }

    return m_upRightButton;
  }

  /**
   * Returns the right (90 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger right() {
    if (m_rightButton == null) {
      m_rightButton = new Trigger(()->m_hid.getPOV(m_povNumber) == 90);
    }

    return m_rightButton;
  }

  /**
   * Returns the downwards-right (135 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger downRight() {
    if (m_downRightButton == null) {
      m_downRightButton = new Trigger(()->m_hid.getPOV(m_povNumber) == 135);
    }

    return m_downRightButton;
  }

  /**
   * Returns the downwards (180 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger down() {
    if (m_downButton == null) {
      m_downButton = new Trigger(()->m_hid.getPOV(m_povNumber) == 180);
    }

    return m_downButton;
  }

  /**
   * Returns the downwards-left (225 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger downLeft() {
    if (m_downLeftButton == null) {
      m_downLeftButton = new Trigger(()->m_hid.getPOV(m_povNumber) == 225);
    }

    return m_downLeftButton;
  }

  /**
   * Returns the left (270 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger left() {
    if (m_leftButton == null) {
      m_leftButton = new Trigger(()->m_hid.getPOV(m_povNumber) == 270);
    }

    return m_leftButton;
  }

  /**
   * Returns the upwards-left (315 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger upLeft() {
    if (m_upLeftButton == null) {
      m_upLeftButton = new Trigger(()->m_hid.getPOV(m_povNumber) == 315);
    }

    return m_upLeftButton;
  }

  /**
   * Returns the center (-1) Trigger object.
   * 
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger center() {
    if (m_centerButton == null) {
      m_centerButton = new Trigger(()->m_hid.getPOV(m_povNumber) == -1);
    }

    return m_centerButton;
  }
}
