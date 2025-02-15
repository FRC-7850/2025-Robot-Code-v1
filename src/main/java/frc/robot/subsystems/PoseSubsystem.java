package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.TransformConstants;;

public class PoseSubsystem extends SubsystemBase{
    ShuffleboardTab VisionTab = Shuffleboard.getTab("Vision Values");
    GenericEntry Camera1Tag; //int
    GenericEntry Camera2Tag; //int
    GenericEntry Camera1TagAmbiguity; //double
    GenericEntry Camera2TagAmbiguity; //double
    GenericEntry RobotPose; //Pose3d

    PhotonCamera camera1 = new PhotonCamera("INSERT CAMERA NAME");
    PhotonCamera camera2 = new PhotonCamera("INSERT OTHER CAMERA NAME");

    //Init Method
    public PoseSubsystem() {
        Camera1Tag = VisionTab.add("Camera 1 Tag", 0).getEntry();
        Camera2Tag = VisionTab.add("Camera 2 Tag", 0).getEntry();
        Camera1TagAmbiguity = VisionTab.add("Camera 1 Ambiguity", 2).withWidget(BuiltInWidgets.kGraph).getEntry();   
        Camera2TagAmbiguity = VisionTab.add("Camera 2 Ambiguity", 2).withWidget(BuiltInWidgets.kGraph).getEntry();
        RobotPose = VisionTab.add("Camera 2 Ambiguity", 99).withWidget(BuiltInWidgets.kField).getEntry();

        int accurateCamera;
        PhotonTrackedTarget accurateTarget;
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        Transform3d cameraToRobot = TransformConstants.kCameraToRobot;
        Pose3d robotPose = new Pose3d(0,0,0,new Rotation3d(0,0,0)); //Initializing with default value

        var camera1Result = camera1.getLatestResult();
        var camera2Result = camera2.getLatestResult();
        //Must check for targets before using results as per API, not unused
        boolean camera1HasTargets = camera1Result.hasTargets();
        boolean camera2HasTargets = camera2Result.hasTargets();
        PhotonTrackedTarget camera1Target = camera1Result.getBestTarget();
        PhotonTrackedTarget camera2Target = camera2Result.getBestTarget();
        int camera1TargetID = camera1Target.getFiducialId();
        int camera2TargetID = camera2Target.getFiducialId();
        double camera1Ambiguity = camera1Target.getPoseAmbiguity();
        double camera2Ambiguity = camera2Target.getPoseAmbiguity();

        if (camera1Target.getPoseAmbiguity() < camera2Target.getPoseAmbiguity()){
            accurateCamera = camera1TargetID;
            accurateTarget = camera1Target;
        }else{
            accurateCamera = camera2TargetID;
            accurateTarget = camera2Target;
        }

        if (aprilTagFieldLayout.getTagPose(accurateCamera).isPresent()) {
          robotPose = PhotonUtils.estimateFieldToRobotAprilTag(accurateTarget.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(accurateTarget.getFiducialId()).get(), cameraToRobot);
        }

       Camera1Tag.setInteger(camera1TargetID);
       Camera1Tag.setInteger(camera2TargetID);
       Camera1TagAmbiguity.setDouble(camera1Ambiguity);
       Camera1TagAmbiguity.setDouble(camera2Ambiguity);
       RobotPose.setValue(robotPose);
    }
}
