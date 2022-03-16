package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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

  public final Trigger topBeamBreakTrigger = new Trigger(this::getIsTopBeamBroken);
  public final Trigger bottomBeamBreakTrigger = new Trigger(this::getIsBottomBeamBroken);
  public final Trigger midtakeStoppedTrigger = new Trigger(this::getIsStopped);

  private int cargoIn = 0;
  private int cargoOut = 0;
  private boolean beamBreakTopBroken;

  @Log
  private boolean isArmed = false;
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
    isArmed = false;
  }

  public void reverse() {
    spin(-Constants.MIDTAKE_LOADING_SPEED);
    isArmed = false;
  }

  public void feed() {
    spin(Constants.MIDTAKE_FEEDING_SPEED);
    isArmed = false;
  }

  public void crawl() {
    spin(Constants.MIDTAKE_CRAWL_SPEED);
    isArmed = false;
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

  public boolean getIsTopBeamClear() {
    return !beamBreakTopBroken;
  }

  /**
   * Returns whether the bottom Beam Break sensor is triggered
   * 
   */
  @Log
  public boolean getIsBottomBeamBroken() {
    return beamBreakBottomBroken;
  }

  public boolean getIsBottomBeamClear() {
    return !beamBreakBottomBroken;
  }

  @Log
  public int getBallCount() {
    if(!beamBreakBottomBroken && !beamBreakTopBroken) {
      return 0;
    }
    else if (beamBreakBottomBroken ^ beamBreakTopBroken) {
      return 1;
    }
    else if (beamBreakBottomBroken && beamBreakTopBroken) {
      return 2;
    }
    else {
      return -1;
    }
  }

  @Log
  public boolean getIsMidtakeFull(){
    return getBallCount() == 2;
  }

  /**
   * Returns whether the color of the ball is the same as your team color
   * 
   * @return
   */
  @Log
  public boolean getIsBallColorCorrect() {
    if(RobotBase.isReal()) {
      // PicoColorSensor.RawColor detectedColor = colorSensor.getRawColor0();
      // boolean isBallRed = detectedColor.red > detectedColor.blue;
      // boolean areWeRed = DriverStation.getAlliance() == Alliance.Red;
      // return isBallRed == areWeRed;
      return true;
    } else {
      return !simSensorInput.getRawButton(4);
    }
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

  public boolean getIsWrongBallDetected() {
    return getColorSensorDetectsBall() && ! getIsBallColorCorrect();
  }

  public boolean getIsStopped() {
    return frontSparkMax.getAppliedOutput() <= 0.05 && backSparkMax.getAppliedOutput() <= 0.05;
  }

  @Log
  public boolean getDidCargoEnter() {
    return lastBeamBreakBottomBroken && !beamBreakBottomBroken;
  }

  @Log  
  public boolean getDidCargoLeave() {
    return lastBeamBreakTopBroken && !beamBreakTopBroken;
  }

  public boolean getIsArmed() {
    return isArmed;
  }

  public void reportArmed() {
    isArmed = true;
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