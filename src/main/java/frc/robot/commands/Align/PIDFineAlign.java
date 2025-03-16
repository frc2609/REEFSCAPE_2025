package frc.robot.commands.Align;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveSubsystem;



public class PIDFineAlign extends Command {
    private PIDController xController, yController, rotController;
    private boolean isLeftScore;
    private Timer dontSeeTagTimer, stopTimer;
    private CommandSwerveDrivetrain drivebase;
    private double tagID = -1;
    private SwerveRequest.RobotCentric m_drive = new SwerveRequest.RobotCentric();
    private double turnP = 0.11;


    public PIDFineAlign(boolean isLeftScore, CommandSwerveDrivetrain drivebase) {
      xController = new PIDController(2, 0.0, 0);  // Vertical movement
      yController = new PIDController(2, 0, 0);  // Horitontal movement
      rotController = new PIDController(turnP, 0, 0);  // Rotation
      this.isLeftScore = isLeftScore;
      this.drivebase = drivebase;
      addRequirements(drivebase);
    }
  
    @Override
    public void initialize() {
      SmartDashboard.putNumber("turn p", turnP);
      this.stopTimer = new Timer();
      this.stopTimer.start();
      this.dontSeeTagTimer = new Timer();
      this.dontSeeTagTimer.start();
      // Left set points
      double Ysetpoint = -0.606;
      double Xsetpoint = -0.225;
      double rotSetPoint = 56.3;

      // Right set points
      if (isLeftScore == false){
        Ysetpoint = -0.238;
        Xsetpoint = -0.232;
        rotSetPoint = 55.3;
      }
      
  
      rotController.setSetpoint(rotSetPoint);
      rotController.setTolerance(1);
  
      xController.setSetpoint(Xsetpoint);
      xController.setTolerance(0.02);

    
      yController.setSetpoint(Ysetpoint);
      yController.setTolerance(0.02);
  
      tagID = LimelightHelpers.getFiducialID("limelight");
    }

  
    @Override
    public void execute() {
      double prevTurnP = turnP;

      turnP = SmartDashboard.getNumber("turn p", turnP);
      if (turnP != prevTurnP){
        rotController.setP(turnP);
      }
      if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == tagID) {
        this.dontSeeTagTimer.reset();
  
        double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight");
        SmartDashboard.putNumber("Fine X val", postions[2]);
  
        double xSpeed = xController.calculate(postions[2]);
        SmartDashboard.putNumber("xspeed", xSpeed);
        double ySpeed = -yController.calculate(postions[0]);
        SmartDashboard.putNumber("Fine Y val", postions[0]);
        double rotValue = -rotController.calculate(postions[4]);
        SmartDashboard.putNumber("Fine rot val", postions[4]);
// drive!
        drivebase.setControl(m_drive
           .withVelocityX(xSpeed) // Drive forward with negative Y(forward)
           .withVelocityY(ySpeed) // Drive left with negative X (left)
           .withRotationalRate(rotValue)
        );


  
        if ( !rotController.atSetpoint()||
              !yController.atSetpoint() ||
              !xController.atSetpoint())
            {
          stopTimer.reset();
        }
      } else {
        drivebase.setControl(m_drive
                .withVelocityX(0) // Drive forward with negative Y(forward)
                .withVelocityY(0) // Drive left with negative X (left)
                .withRotationalRate(0));
      }
  
      SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }
  
    @Override
    public void end(boolean interrupted) {
        drivebase.setControl(m_drive
        .withVelocityX(0) // Drive forward with negative Y(forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0));
    }
  
    @Override
    public boolean isFinished() {
      SmartDashboard.putBoolean("Done ", 
        this.dontSeeTagTimer.hasElapsed(1) || stopTimer.hasElapsed(0.3));
      // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
      return this.dontSeeTagTimer.hasElapsed(1) ||
          stopTimer.hasElapsed(0.3);
    }
  }