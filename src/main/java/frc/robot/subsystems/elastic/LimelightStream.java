package frc.robot.subsystems.elastic;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightStream extends SubsystemBase {
    public LimelightStream() {
        HttpCamera limelightCamera = new HttpCamera("LimelightCamera", "http://limelight-april:5801/stream.mjpg");

        limelightCamera.setResolution(320, 240); // Set the desired resolution
    }
}