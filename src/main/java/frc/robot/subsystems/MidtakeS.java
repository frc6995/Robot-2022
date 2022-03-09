package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.color.PicoColorSensor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Takes balls from the intake and holds it and sends it to the shooter.
 * 
 * @authors Jonas An and Ben Su
 */
public class MidtakeS extends SubsystemBase implements Loggable{
  private GenericHID simSensorInput = new GenericHID(5);
  /** Creates a new IntakeS. */
  @Log(methodName = "getAppliedOutput", name = "frontVolts")
  private CANSparkMax frontSparkMax = new CANSparkMax(Constants.CAN_ID_MIDTAKE_FRONT,
      MotorType.kBrushless);
  private RelativeEncoder frontSparkMaxEncoder = frontSparkMax.getEncoder();
  @Log(methodName = "getAppliedOutput", name = "backVolts")
  private CANSparkMax backSparkMax = new CANSparkMax(Constants.CAN_ID_MIDTAKE_BACK,
      MotorType.kBrushless);
  private RelativeEncoder backSparkMaxEncoder = backSparkMax.getEncoder();
  private DigitalInput beamBreakTop = new DigitalInput(Constants.BEAM_BREAK_TOP_PORT_NUMBER);
  @Log
  private boolean lastBeamBreakTopBroken = getIsTopBeamBroken();

  private DigitalInput beamBreakBottom = new DigitalInput(Constants.BEAM_BREAK_BOTTOM_PORT_NUMBER);
  private boolean beamBreakBottomBroken = false;
  @Log
  private boolean lastBeamBreakBottomBroken = getIsBottomBeamBroken();
  private final PicoColorSensor colorSensor = new PicoColorSensor();

  public final Trigger goingForwardTrigger = new Trigger(this::getIsGoingBackward).negate();
  @Log(methodName = "getAsBoolean", name = "cargoEnteredTrigger")
  public final Trigger cargoEnteredTrigger = new Trigger(this::getDidCargoEnter);
  @Log(methodName = "getAsBoolean", name = "cargoLeftTrigger")
  public final Trigger cargoLeftTrigger = new Trigger(this::getDidCargoLeave);

  private int cargoIn = 0;
  private int cargoOut = 0;
  private boolean beamBreakTopBroken;
  
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

  public void reverse() {
    spin(-Constants.MIDTAKE_LOADING_SPEED);
  }

  public void feed() {
    spin(Constants.MIDTAKE_FEEDING_SPEED);
  }

  public void crawl() {
    spin(Constants.MIDTAKE_CRAWL_SPEED);
  }

  public void stop() {
    spin(0);
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
    return beamBreakTopBroken;
  }

  /**
   * Returns whether the bottom Beam Break sensor is triggered
   * 
   */
  @Log
  public boolean getIsBottomBeamBroken() {
    return beamBreakBottomBroken;
  }

  /**
   * Returns whether the color of the ball is the same as your team color
   * 
   * @return
   */
  @Log
  public boolean getIsBallColorCorrect() {
    // if(RobotBase.isReal()) {
    //   PicoColorSensor.RawColor detectedColor = colorSensor.getRawColor0();
    //   boolean isBallRed = detectedColor.red > detectedColor.blue;
    //   boolean areWeRed = DriverStation.getAlliance() == Alliance.Red;
    //   return isBallRed == areWeRed;
    // } else {
    //   return !simSensorInput.getRawButton(4);
    // }
    return true;
  }

  /**
   * Returns whether the color sensor's proximity sensor detects a ball.
   */
  @Log
  public boolean getColorSensorDetectsBall() {
    if(RobotBase.isReal()) {
      return colorSensor.getProximity0() > Constants.COLOR_SENSOR_PROXIMITY_THRESHOLD;
    }
    else{
      return simSensorInput.getRawButton(3);
    }
  }

  public boolean getIsGoingBackward() {
    return frontSparkMax.getAppliedOutput() < -0.01 && backSparkMax.getAppliedOutput() < -0.01;
  }

  @Log
  public boolean getDidCargoEnter() {
    return lastBeamBreakBottomBroken && !beamBreakBottomBroken;
  }

  @Log  
  public boolean getDidCargoLeave() {
    return lastBeamBreakTopBroken && !beamBreakTopBroken;
  }



  public void resetCargoCount(int cargoOut, int storedCargo) {
    cargoIn = cargoOut + storedCargo;
    this.cargoOut = cargoOut;
  }

  public void resetCargoCount() {
    resetCargoCount(0, 0);
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
    if(DriverStation.isDisabled()) {
      stop();
    }
    if(getDidCargoEnter()) { 
      cargoIn++;
    }
    if(getDidCargoLeave()) { 
      cargoOut++;
    }
    

    lastBeamBreakBottomBroken = beamBreakBottomBroken;
    lastBeamBreakTopBroken = beamBreakTopBroken;

    if(RobotBase.isReal()) {
      beamBreakBottomBroken = !beamBreakBottom.get();
      beamBreakTopBroken = !beamBreakTop.get();
    }
    else {
      beamBreakBottomBroken =  simSensorInput.getRawButton(1);
      beamBreakTopBroken = simSensorInput.getRawButton(2);
    }
  }
}