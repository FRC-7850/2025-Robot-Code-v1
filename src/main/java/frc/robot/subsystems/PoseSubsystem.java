package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.TransformConstants;;

public class PoseSubsystem extends SubsystemBase{
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    LTVUnicycleController controller = new LTVUnicycleController(VecBuilder.fill(0.0625, 0.125, 2.0), VecBuilder.fill(1.0, 2.0), 0.02, 9);
    ChassisSpeeds LTVControlOutput; 
    ShuffleboardTab VisionTab = Shuffleboard.getTab("Vision Values");
    GenericEntry Camera1Tag; //int
    GenericEntry Camera2Tag; //int
    GenericEntry Camera1TagAmbiguity; //double
    GenericEntry Camera2TagAmbiguity; //double
    GenericEntry RobotPose; //Pose3d

    PhotonCamera camera1 = new PhotonCamera("INSERT CAMERA NAME");
    PhotonCamera camera2 = new PhotonCamera("INSERT OTHER CAMERA NAME");
    
    Pose3d PoseFromAprilTag(PhotonTrackedTarget target, Transform3d cameraToRobot) {
        return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
    }

    //Init Method
    public PoseSubsystem() {
        Camera1Tag = VisionTab.add("Camera 1 Tag", 0).getEntry();
        Camera2Tag = VisionTab.add("Camera 2 Tag", 0).getEntry();
        Camera1TagAmbiguity = VisionTab.add("Camera 1 Ambiguity", 2).withWidget(BuiltInWidgets.kGraph).getEntry();   
        Camera2TagAmbiguity = VisionTab.add("Camera 2 Ambiguity", 2).withWidget(BuiltInWidgets.kGraph).getEntry();
        RobotPose = VisionTab.add("Camera 2 Ambiguity", 99).withWidget(BuiltInWidgets.kField).getEntry();

        Transform3d cameraToRobot = TransformConstants.kCameraToRobot;
        Pose3d robotPoseFromCamera1 = new Pose3d();
        Pose3d robotPoseFromCamera2 = new Pose3d();
        Pose2d robotPose2d = new Pose2d();

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

    if (camera1HasTargets && camera2HasTargets){
        //Combining results for better pose
         robotPoseFromCamera1 = PoseFromAprilTag(camera1Target, cameraToRobot);
         robotPoseFromCamera2 = PoseFromAprilTag(camera2Target, cameraToRobot);
         robotPose2d = new Pose2d(
            (robotPoseFromCamera1.getX() + robotPoseFromCamera2.getX() / 2), 
            (robotPoseFromCamera1.getY() + robotPoseFromCamera2.getY() / 2),
            new Rotation2d(
                (robotPoseFromCamera1.getRotation().getX() + robotPoseFromCamera1.getRotation().getX() / 2),
                (robotPoseFromCamera1.getRotation().getY() + robotPoseFromCamera1.getRotation().getY() / 2)
            )
         );
    } else if(camera1HasTargets){
        robotPoseFromCamera1 = PoseFromAprilTag(camera1Target, cameraToRobot);
        robotPose2d = new Pose2d(
            robotPoseFromCamera1.getX(),
            robotPoseFromCamera1.getY(),
            new Rotation2d(
                robotPoseFromCamera1.getRotation().getX(),
                robotPoseFromCamera1.getRotation().getY()
            )
        );
    } else if(camera2HasTargets){
        robotPoseFromCamera2 = PoseFromAprilTag(camera2Target, cameraToRobot);
        robotPose2d = new Pose2d(
            robotPoseFromCamera2.getX(),
            robotPoseFromCamera2.getY(),
            new Rotation2d(
                robotPoseFromCamera2.getRotation().getX(),
                robotPoseFromCamera2.getRotation().getY()
            )
        );
    }

    LTVControlOutput = controller.calculate(robotPose2d, null);

    Camera1Tag.setInteger(camera1TargetID);
    Camera1Tag.setInteger(camera2TargetID);
    Camera1TagAmbiguity.setDouble(camera1Ambiguity);
    Camera1TagAmbiguity.setDouble(camera2Ambiguity);
    RobotPose.setValue(robotPose2d);
    }

    public ChassisSpeeds GetLTVControlOutput(){
        return LTVControlOutput;
    }
}
