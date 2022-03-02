package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Takes balls from the intake and holds it and sends it to the shooter.
 * 
 * @authors Jonas An and Ben Su
 */
public class MidtakeS extends SubsystemBase implements Loggable{
  /** Creates a new IntakeS. */
  private CANSparkMax frontSparkMax = new CANSparkMax(Constants.CAN_ID_MIDTAKE_FRONT,
      MotorType.kBrushless);
  private RelativeEncoder frontSparkMaxEncoder = frontSparkMax.getEncoder();
  private CANSparkMax backSparkMax = new CANSparkMax(Constants.CAN_ID_MIDTAKE_BACK,
      MotorType.kBrushless);
  private RelativeEncoder backSparkMaxEncoder = backSparkMax.getEncoder();
  private DigitalInput beamBreakTop = new DigitalInput(Constants.BEAM_BREAK_TOP_PORT_NUMBER);
  private boolean lastBeamBreakTopBroken = getIsTopBeamBroken();
  private DigitalInput beamBreakBottom = new DigitalInput(Constants.BEAM_BREAK_BOTTOM_PORT_NUMBER);
  private boolean lastBeamBreakBottomBroken = getIsBottomBeamBroken();
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  public final Trigger goingForwardTrigger = new Trigger(this::getIsGoingBackward).negate();
  public final Trigger cargoEnteredTrigger = new Trigger(this::getDidCargoEnter);
  public final Trigger cargoLeftTrigger = new Trigger(this::getDidCargoLeave);

  private int cargoIn = 0;
  private int cargoOut = 0;

  /**
   * Create a new MidtakeS
   */
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
    frontSparkMax.setVoltage(frontSpeed * 12);
  }

  /**
   * Sets the speed of the back motor
   * 
   * @param backSpeed the back speed
   */
  public void setBackSpeed(double backSpeed) {
    backSparkMax.setVoltage(backSpeed * 12);
  }

  /**
   * Spins both midtake motors at set speed.
   */
  public void load() {
    spin(Constants.MIDTAKE_LOADING_SPEED);
  }

  public void feed() {
    spin(Constants.MIDTAKE_FEEDING_SPEED);
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
  @Log
  public boolean getIsTopBeamBroken() {
    return !beamBreakTop.get();
  }

  /**
   * Returns whether the bottom Beam Break sensor is triggered
   * 
   */
  @Log
  public boolean getIsBottomBeamBroken() {
    return !beamBreakBottom.get();
  }

  /**
   * Returns whether the color of the ball is the same as your team color
   * 
   * @return
   */
  @Log
  public boolean getIsBallColorCorrect() {
    Color detectedColor = colorSensor.getColor();
    boolean isBallRed = detectedColor.red > detectedColor.blue;
    boolean areWeRed = DriverStation.getAlliance() == Alliance.Red;
    return isBallRed == areWeRed;
  }

  /**
   * Returns whether the color sensor's proximity sensor detects a ball.
   */
  @Log
  public boolean getColorSensorDetectsBall() {
    return colorSensor.getProximity() > Constants.COLOR_SENSOR_PROXIMITY_THRESHOLD;
  }

  /**
   * Returns true only when the top beam break has just become unbroken and the midtake is feeding toward the shooter.
   */


  public boolean getIsGoingBackward() {
    return frontSparkMaxEncoder.getVelocity() < -0.01 && backSparkMaxEncoder.getVelocity() < -0.01;
  }

  public boolean getDidCargoEnter() {
    return lastBeamBreakBottomBroken && !getIsBottomBeamBroken() && !getIsGoingBackward();
  }
  
  public boolean getDidCargoLeave() {
    return lastBeamBreakTopBroken && !getIsTopBeamBroken() && !getIsGoingBackward();
  }

  /**
   * Stops the midtake.
   */

  public void stop() {
    spin(0);
  }

  public void resetCargoCount(int storedCargo) {
    cargoIn = storedCargo;
    cargoOut = 0;
  }

  public void resetCargoCount() {
    resetCargoCount(0);
  }

  @Log
  public int getStoredCargo() {
    return cargoIn - cargoOut;
  }

  @Log
  public int getCargoOutCount() {
    return cargoOut;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // If the midtake is driving toward the shooter and the bottom sensor is falling
    if(getDidCargoEnter()) { 
      cargoIn++;
    }
    if(getDidCargoLeave()) { 
      cargoOut++;
    }
    lastBeamBreakBottomBroken = getIsBottomBeamBroken();
    lastBeamBreakTopBroken = getIsTopBeamBroken();
  }
}