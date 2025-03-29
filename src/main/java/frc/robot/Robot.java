// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import com.pathplanner.lib.commands.PathfindingCommand;

import frc.robot.utils.LimelightHelpers;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    Thread m_visionThread;

    private final RobotContainer m_robotContainer;
    // private final DigitalSource sourcePWMX = new DigitalInput(9);
    // private final DutyCycle dutyPWMX = new DutyCycle(sourcePWMX);

    public Robot() {
        Thread m_visionThread = new Thread(
            () -> {
                UsbCamera camera = CameraServer.startAutomaticCapture();
                camera.setResolution(640, 480);
            }
        );
        m_visionThread.setDaemon(true);
        m_visionThread.start();

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        double[] rightPositions = LimelightHelpers.getBotPose_TargetSpace("limelight");
        double[] leftPositions = LimelightHelpers.getBotPose_TargetSpace("limelight-intake");
        
        if (rightPositions.length > 4) {
            SmartDashboard.putNumber("Right X val", rightPositions[2]);
            SmartDashboard.putNumber("Right Y val", rightPositions[0]);
            SmartDashboard.putNumber("Right rot val", rightPositions[4]);
        }

        if (leftPositions.length > 4) {
            SmartDashboard.putNumber("Left X val", leftPositions[2]);
            SmartDashboard.putNumber("Left Y val", leftPositions[0]);
            SmartDashboard.putNumber("Left rot val", leftPositions[4]);
        }
    }

    @Override
    public void robotInit() {
        DataLogManager.start();
        PathfindingCommand.warmupCommand().schedule();
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
