// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  Thread m_visionThread;

  private final RobotContainer m_robotContainer;
  // private final DigitalSource sourcePWMX = new DigitalInput(9);
  // private final DutyCycle dutyPWMX = new DutyCycle(sourcePWMX);

  public Robot() {
      m_visionThread = new Thread(
      () -> {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);
      }
    );
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  
    

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight");
    if (postions.length > 4){
      SmartDashboard.putNumber("Fine X val", postions[2]);
      SmartDashboard.putNumber("Fine Y val", postions[0]);
      SmartDashboard.putNumber("Fine rot val", postions[4]);
    }
    SmartDashboard.putBoolean("retracted", m_robotContainer.intakeFlop.retractedTrigger.getAsBoolean());
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() {
    DataLogManager.start();
    PathfindingCommand.warmupCommand().schedule();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    
  }





  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    // m_robotContainer.climber.goToPosition(-50);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.climber.goToPosition(-50);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
