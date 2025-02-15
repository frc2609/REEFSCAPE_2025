package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.VisionSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.configs.jni.ConfigJNI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.*;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.Constants.VisionConstants;
import org.json.simple.JSONObject;

import java.io.Console;
import java.io.FileReader;
import java.io.IOException;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class AlignCommand extends Command {
    private final CommandSwerveDrivetrain m_Swerve;
    private final Limelight m_limelight;
    private final Pigeon2 m_Pigeon2;
    private Pose2d taPose2d;
    private JSONObject map;

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
   .withDeadband(4.73 * 0.1).withRotationalDeadband(2 * 0.1); // Add a 10% deadband




    public AlignCommand(CommandSwerveDrivetrain swerve, Limelight limelight, Pigeon2 pidgey) {
        m_Swerve = swerve;
        m_limelight = limelight;
        m_Pigeon2 = pidgey;
        String fileLocation =  String.format("%s%s", Filesystem.getDeployDirectory(), "/frc2025r2.json");
        SmartDashboard.putString("print", fileLocation);
        try (FileReader reader = new FileReader(fileLocation)) {
            JSONParser jsonParser = new JSONParser();
            map = (JSONObject) jsonParser.parse(reader);
            SmartDashboard.putString("map", map.toJSONString());
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
        addRequirements(swerve, limelight);
    }



    private double limelightAimProportional() {
        // kP (constant of proportionality)
        // Determines the aggressiveness of the proportional control loop
        double kP = 0.04;
    
        // Get the "tx" value from the Limelight
        double targetingAngularVelocity = m_limelight.get_tx() * kP;

        SmartDashboard.putNumber("limelightX: ", m_limelight.get_tx());

    
        // Convert to radians per second for the drivetrain
        targetingAngularVelocity *= -TunerConstants.kSpeedAt12Volts.magnitude();
    
        // Invert since tx is positive when the target is to the right of the crosshair
        // targetingAngularVelocity *= 1.0;
    
        return targetingAngularVelocity;
    }
    
    // Proportional ranging control with Limelight's "ty" value
    // Works best if the Limelight's mount height and target mount height are different.
    private double limelightRangeProportional() {
        double kP = 0.06;
    
        // Get the "ty" value from the Limelight
        // double targetingForwardSpeed = m_Vision.getTY() * kP;
        double targetingForwardSpeed = m_limelight.get_ty() * kP;


    
        // Convert to meters per second for the drivetrain
        targetingForwardSpeed *= -TunerConstants.kSpeedAt12Volts.magnitude();
    
        // Invert the direction for proper control
        // targetingForwardSpeed *= 1.0;
    
        return targetingForwardSpeed;
    }
    // private double limelightRotProportional() {
    //     double kP = 0.06;
    
    //     // Get the "ty" value from the Limelight
    //     // double targetingForwardSpeed = m_Vision.getTY() * kP;
    //     double targetingForwardSpeed = m_limelight.get_tl() * kP;

    //     //SmartDashboard.putNumber("limelightX", LimelightHelpers.getTY("limelight"));
    
    //     // Convert to meters per second for the drivetrain
    //     targetingForwardSpeed *= TunerConstants.kSpeedAt12Volts.magnitude();
    
    //     // Invert the direction for proper control
    //     // targetingForwardSpeed *= 1.0;
    
    //     return targetingForwardSpeed;
    // }

    private double LimelightRoation(){
        double kP = 0.06;
        double angle = m_Pigeon2.getAccumGyroY().getValueAsDouble();
        SmartDashboard.putNumber("Angle",angle);
        return angle;

    }

    public void execute(){
        // double rot = limelightAimProportional();


        // Store the ID of the AprilTag the Limelight is seeing
        // double tagID = LimelightHelpers.getFiducialID("limelight-seaweed");
        // double xSpeed = limelightRangeProportional(); 
        // double yspeed = limelightAimProportional();


        
        //System.out.print(map);
        // SmartDashboard.putString("AprilTagID", map.get("1").toString());

        // double tagYaw = LimelightHelpers.getBotPose_wpiBlue("limelight-seaweed")[5];



        /*m_Swerve.setControl(
            m_driveRequest
                .withVelocityX(xSpeed)  
                .withVelocityY(yspeed)
                
        );*/

        //m_Swerve.applyRequest(()->m_driveRequest.withVelocityX(xSpeed));
    }
}