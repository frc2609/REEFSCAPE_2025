package frc.robot.subsystems.elastic;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CameraDisplay extends SubsystemBase {

    public CameraDisplay() {
        UsbCamera camera = CameraServer.startAutomaticCapture(0);
        camera.setResolution(320, 240); // Set the desired resolution
    }
  


  }