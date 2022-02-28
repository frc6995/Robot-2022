package frc.robot.util.interpolation;

// Shot parameter
public class ShotParameter {

    // Variables
    public final double backWheelRpm;
    public final double frontWheelRpm;
    public final double timeOfFlight;

    // Constructor
    public ShotParameter(double frontWheelRpm, double backWheelRpm, double timeOfFlight) {
        this.frontWheelRpm = frontWheelRpm;
        this.backWheelRpm = backWheelRpm;
        this.timeOfFlight = timeOfFlight;
    }   

    // Method equals
    public boolean equals(ShotParameter other) {
        return Math.abs(this.frontWheelRpm - other.frontWheelRpm) < 0.1 &&
        Math.abs(this.backWheelRpm - other.backWheelRpm) < 0.1 &&
        Math.abs(this.timeOfFlight - other.timeOfFlight) < 0.1;
    }

    // Method to interpolate
    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(frontWheelRpm, end.frontWheelRpm, t), 
            lerp(backWheelRpm, end.backWheelRpm, t), 
            lerp(timeOfFlight, end.timeOfFlight, t)
        );
    }

    // Method lerp
    private double lerp(double y2, double y1, double t) {
        return y1 + (t * (y2 - y1));
    }
 
}