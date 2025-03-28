package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightHelpers;

public class Limelight extends SubsystemBase{
    private String name;
    public Limelight(String name){
        this.name = name;
    }

    public double get_ty(){
        return LimelightHelpers.getTY(this.name);
    }

    public double get_tx(){
        return LimelightHelpers.getTX(this.name);
    }
    public boolean get_tv(){
        return LimelightHelpers.getTV(this.name);
    }

    public Pose2d get_Pose2d(){
        //SmartDashboard.putString("passs", "it enters here");
        //System.out.print("enters here");
        double ID = LimelightHelpers.getFiducialID(this.name);
        //System.out.println((int)ID);
        //SmartDashboard.putNumber("ID", ID);
        // JSONObject fiducial = (JSONObject) fiducials.get((int)ID);
        // JSONArray transform = (JSONArray) fiducial.get("transform");
        //System.out.println(Constants.VisionConstants.aprilTagMap.size());
        double[] transform = Constants.VisionConstants.aprilTagMap.get((int)ID);
        double yaw = Math.atan2((double)transform[1], (double)transform[0]);
        Pose2d pose = new Pose2d((double)transform[3], (double)transform[7], new Rotation2d(yaw));
        //SmartDashboard.putString("pose", pose.toString());
        return pose;
    

    }
    
    // public double get_yaw(){
    //     LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.name);
        
    //     return mt1.pose.getRotation().getDegrees();
    // }


    // @Override
    // public void periodic(){
    //     // SmartDashboard.putNumber("get yaw", get_yaw());
    // }

}