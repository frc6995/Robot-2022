package edu.wpi.first.wpilibj2.command;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.EnumMap;

/** Provides Triggers for binding commands to any GenericHID inherited class. */
public class CommandControllerPOV {
  private final GenericHID m_hid;
  private final int m_povNumber;

  private enum POVAngle {
    kCenter(-1),
    kUp(0),
    kUpRight(45),
    kRight(90),
    kDownRight(135),
    kDown(180),
    kDownLeft(225),
    kLeft(270),
    kUpLeft(315);

    @SuppressWarnings("MemberName")
    public final int value;

    POVAngle(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the POV angle. This is done by stripping the leading `k`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the angle.
     */
    @Override
    public String toString() {
      return this.name().substring(1); // Remove leading `k`
    }
  }

  private final EnumMap<POVAngle, Trigger> m_povs = new EnumMap<>(POVAngle.class);

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
   * Builds a {@link Trigger} for this POV from the provided {@link POVAngle}.
   *
   * @param button the POVAngle to build for
   * @return Built Trigger
   */
  private Trigger build(POVAngle angle) {
    return new Trigger(()->{return m_hid.getPOV(m_povNumber) == angle.value;});
  }

  /**
   * Returns the centered (not pressed) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger center() {
    return m_povs.computeIfAbsent(POVAngle.kCenter, this::build);
  }

  /**
   * Returns the upper (0 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger up() {
    return m_povs.computeIfAbsent(POVAngle.kUp, this::build);
  }

  /**
   * Returns the upper-right (45 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger upRight() {
    return m_povs.computeIfAbsent(POVAngle.kUpRight, this::build);
  }

  /**
   * Returns the right (90 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger right() {
    return m_povs.computeIfAbsent(POVAngle.kRight, this::build);
  }

  /**
   * Returns the downwards-right (135 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger downRight() {
    return m_povs.computeIfAbsent(POVAngle.kDownRight, this::build);
  }

  /**
   * Returns the downwards (180 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger down() {
    return m_povs.computeIfAbsent(POVAngle.kDown, this::build);
  }

  /**
   * Returns the downwards-left (225 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger downLeft() {
    return m_povs.computeIfAbsent(POVAngle.kDownLeft, this::build);
  }

  /**
   * Returns the left (270 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger left() {
    return m_povs.computeIfAbsent(POVAngle.kLeft, this::build);
  }

  /**
   * Returns the upwards-left (315 degrees) Trigger object.
   *
   * <p>To get its value, use {@link Trigger#get()}.
   */
  public Trigger upLeft() {
    return m_povs.computeIfAbsent(POVAngle.kUpLeft, this::build);
  }
}