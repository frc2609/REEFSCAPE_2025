package frc.robot.subsystems.elastic;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightStream extends SubsystemBase {
    @SuppressWarnings("resource")
    public LimelightStream() {
        HttpCamera limelightCamera = new HttpCamera("LimelightCamera", "http://limelight-intake:5801/stream.mjpg");

        limelightCamera.setResolution(320, 240); // Set the desired resolution
    }
}