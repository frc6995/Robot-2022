package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** Add your docs here. */
public class OdometryManager implements Loggable {
    @Log(name="transformX", methodName = "getX")
    Transform2d m_robotToHubTrans;
    Transform2d m_robotToTurretTrans;
    Transform2d m_robotToCameraTrans;
    @Log(name = "currX", methodName = "getX")
    Pose2d m_currPoseMeters;
    @Log(name = "lastX", methodName = "getX")
    Pose2d m_lastPoseMeters;

    Supplier<Pose2d> poseSupplier;
    Supplier<Rotation2d> headingSupplier;
    Supplier<Rotation2d> turretAngleSupplier;
    Rotation2d offsetAroundHub = new Rotation2d();
    Consumer<Double> transformAngularVelocityConsumer;
    private Transform2d m_cameraToHubTrans;
    private Transform2d m_lastRobotToHubTrans;
    private Transform2d m_fieldToRobotZeroTrans;

    private boolean isVisionEnabled = false;



    

    public OdometryManager(Supplier<Pose2d> currentPose,
        Supplier<Rotation2d> headingSupplier,
        Supplier<Rotation2d> turretAngle,

        Consumer<Double> transformAngularVelocityConsumer) {
        poseSupplier = currentPose;
        this.headingSupplier = headingSupplier;
        turretAngleSupplier = ()->{return turretAngle.get().rotateBy(Constants.ROBOT_TO_TURRET_ZERO_ROT);};
        this.transformAngularVelocityConsumer = transformAngularVelocityConsumer;

        
        m_currPoseMeters = currentPose.get();
        m_lastPoseMeters = m_currPoseMeters;
        getRobotToCamera();
        m_robotToHubTrans = new Transform2d(m_currPoseMeters, Constants.HUB_CENTER_POSE);
        m_cameraToHubTrans = m_robotToCameraTrans.inverse().plus(m_robotToHubTrans);
        m_lastRobotToHubTrans = m_robotToHubTrans;
    }

    public void setOffsetAroundHub(Rotation2d offset) {
        offsetAroundHub = offset;
    }

    public Rotation2d getOffsetAroundHub() {
        return offsetAroundHub;
    }

    public void setVisionEnabled(boolean enabled) {
        isVisionEnabled = enabled;
    }

    public boolean getVisionEnabled() {
        return isVisionEnabled;
    }

    public void periodic(){
        m_robotToTurretTrans = new Transform2d(
            new Translation2d(), turretAngleSupplier.get()
          );
        getRobotToCamera();
        updateOdometry();
    }

    public Transform2d getRobotToTurret() {
        return m_robotToTurretTrans;
    }

    public Transform2d getRobotToCamera() {
        m_robotToCameraTrans = new Transform2d(
            new Translation2d(Constants.CAMERA_CENTER_OFFSET,0).rotateBy(turretAngleSupplier.get()), turretAngleSupplier.get()
        );
        return m_robotToCameraTrans;
    }

    public Pose2d getTargetPose() {
        return m_currPoseMeters
                .transformBy(m_robotToHubTrans);
    }

    public Transform2d getRobotToHub() {
        return m_robotToHubTrans;
    }

    public Transform2d getCameraToHub() {
        return m_cameraToHubTrans;
    }

    public Pose2d getEstimatedRobotPose() {
        return Constants.HUB_CENTER_POSE.transformBy(
            new Transform2d(
                new Translation2d(
                    getDistanceToCenter(),
                    new Rotation2d(Math.PI).plus(getRotationOffset()).plus(headingSupplier.get()).plus(offsetAroundHub)
                ), headingSupplier.get().plus(offsetAroundHub)));
    }

    /**
     * Read the pose from the Field2d widget and treat it as a drivetrain odometry update.
     */
    public void updateOdometry() {
        updateOdometry(poseSupplier.get());
    }

    public void updateOdometry(Pose2d newPose) {
        m_currPoseMeters = newPose;
        updateTargetOffset(m_currPoseMeters);
        m_lastPoseMeters = m_currPoseMeters;
        m_lastRobotToHubTrans = m_robotToHubTrans;
    }

    public Pose2d getCurrentRobotPose() {
        return m_currPoseMeters;
    }

    public static Rotation2d getDirection(Transform2d transform) {
        return getDirection(transform.getTranslation());
    }

    public static Rotation2d getDirection(Translation2d transform) {
        return new Rotation2d(Math.atan2(transform.getY(), transform.getX()));
    }
    /**
     * Update the robot-relative target offset. 
     * @param newPose the new robot pose to update against.
     */
    public void updateTargetOffset(Pose2d newPose) {
        Transform2d delta = new Transform2d(m_lastPoseMeters, newPose); // Translation then rotation to get to the new pose from the last pose.
        
        Translation2d deltaTranslation = new Translation2d(
            m_robotToHubTrans.getX() - delta.getX(),
            m_robotToHubTrans.getY() - delta.getY()
        ).rotateBy(delta.getRotation().unaryMinus());
        Rotation2d deltaRotation = m_lastPoseMeters.getRotation().plus(delta.getRotation()).unaryMinus();
        m_robotToHubTrans =
            new Transform2d(
                deltaTranslation, deltaRotation  
            );
        transformAngularVelocityConsumer.accept(
            getDirection(m_robotToHubTrans).minus(getDirection(m_lastRobotToHubTrans)).getRadians() / 0.02);
    }

    /**
     * Adjusts the transform to the hub based on a vision measurement,
     * including the distance from the center of the robot to the vision tape,
     * and the radian offset between the robot heading and the target.
     * @param distanceToTapeMeters
     * @param rotationOffsetRadians
     */
    public void addVisionMeasurement(Transform2d cameraToHubTrans) {
        if(isVisionEnabled) {
            m_cameraToHubTrans = cameraToHubTrans;
            Transform2d robotToTargetTrans = m_robotToCameraTrans.plus(cameraToHubTrans);
            m_robotToHubTrans = robotToTargetTrans;
        }
    }

    /**
     * Get the distance to the center of the hub.
     * @return the distance to the center of the hub.
     */
    @Log
    public double getDistanceToCenter() {
        return m_robotToHubTrans.getTranslation().getNorm();
    }

    public boolean getDistanceInRange() {
        return (Constants.SHOOTER_DISTANCES[0] < getDistanceToCenter() && (getDistanceToCenter() < Constants.SHOOTER_DISTANCES[Constants.SHOOTER_DISTANCES.length - 1]));
    }

    /**
     * Get the rate of change of the transform's heading offset, in radians per second.
     */

    public double getTransformAngularVelocity() {
        return (getDirection(m_lastRobotToHubTrans.plus(m_robotToHubTrans)).getRadians()) / 0.02 /*dt*/;
    }
    
    /**
     * Get the rotation transform that would transform the robot to be pointing at the hub.
     */
    @Log(methodName = "getRadians")
    public Rotation2d getRotationOffset() {

        return getDirection(m_robotToHubTrans);
    }
}
