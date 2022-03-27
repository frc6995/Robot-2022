package edu.wpi.first.wpilibj2.command;

import static edu.wpi.first.wpilibj.XboxController.Axis;
import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.EnumMap;

/**
 * Provides Triggers for binding commands to an XboxController's buttons. Additionally offers
 * getters for retrieving axis values.
 */
public class CommandXboxController extends GenericHID {
  // reuses the Button and Axis enums from the original XboxController

  private final EnumMap<Button, Trigger> m_buttons = new EnumMap<>(Button.class);

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

    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }

  /**
   * Builds a {@link Trigger} for this controller from the provided {@link Button}.
   *
   * @param button the Button to build for
   * @return Built Trigger
   */
  private Trigger build(Button button) {
    return new Trigger(()->this.getRawButton(button.value));
  }

  /**
   * Returns the left bumper's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger leftBumper() {
    return m_buttons.computeIfAbsent(Button.kLeftBumper, this::build);
  }

  /**
   * Returns the right bumper's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger rightBumper() {
    return m_buttons.computeIfAbsent(Button.kRightBumper, this::build);
  }

  /**
   * Returns the left stick's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger leftStick() {
    return m_buttons.computeIfAbsent(Button.kLeftStick, this::build);
  }

  /**
   * Returns the right stick's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger rightStick() {
    return m_buttons.computeIfAbsent(Button.kRightStick, this::build);
  }

  /**
   * Returns the A button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  @SuppressWarnings("checkstyle:MethodName")
  public Trigger a() {
    return m_buttons.computeIfAbsent(Button.kA, this::build);
  }

  /**
   * Returns the B button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  @SuppressWarnings("checkstyle:MethodName")
  public Trigger b() {
    return m_buttons.computeIfAbsent(Button.kB, this::build);
  }

  /**
   * Returns the X button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  @SuppressWarnings("checkstyle:MethodName")
  public Trigger x() {
    return m_buttons.computeIfAbsent(Button.kX, this::build);
  }

  /**
   * Returns the Y button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  @SuppressWarnings("checkstyle:MethodName")
  public Trigger y() {
    return m_buttons.computeIfAbsent(Button.kY, this::build);
  }

  /**
   * Returns the back button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger back() {
    return m_buttons.computeIfAbsent(Button.kBack, this::build);
  }

  /**
   * Returns the start button's Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger start() {
    return m_buttons.computeIfAbsent(Button.kStart, this::build);
  }

  /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return getRawAxis(Axis.kLeftX.value);
  }

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightX() {
    return getRawAxis(Axis.kRightX.value);
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return getRawAxis(Axis.kLeftY.value);
  }

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightY() {
    return getRawAxis(Axis.kRightY.value);
  }

  /**
   * Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getLeftTriggerAxis() {
    return getRawAxis(Axis.kLeftTrigger.value);
  }

  /**
   * Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getRightTriggerAxis() {
    return getRawAxis(Axis.kRightTrigger.value);
  }
}