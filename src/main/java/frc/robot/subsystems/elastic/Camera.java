package frc.robot.subsystems.elastic;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;


public class Camera extends SubsystemBase {

    public Camera() {
        UsbCamera camera = CameraServer.startAutomaticCapture(0);
        camera.setResolution(320, 240); // Set the desired resolution
    }
  


  }