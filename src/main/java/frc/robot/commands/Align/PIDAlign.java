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



public class PIDAlign extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private CommandSwerveDrivetrain drivebase;
    private double tagID = -1;
    private SwerveRequest.RobotCentric m_drive = new SwerveRequest.RobotCentric();
    private double offset;


    public PIDAlign(boolean isRightScore, CommandSwerveDrivetrain drivebase, double offset) {
      xController = new PIDController(1.75, 0.0, 0);  // Vertical movement
     yController = new PIDController(1.5, 0.0, 0);  // Horitontal movement
      rotController = new PIDController(0.1, 0, 0);  // Rotation
      this.isRightScore = isRightScore;
      this.drivebase = drivebase;
      this.offset = offset;// meausered in meters
      addRequirements(drivebase);
    }
  
    @Override
    public void initialize() {
      this.stopTimer = new Timer();
      this.stopTimer.start();
      this.dontSeeTagTimer = new Timer();
      this.dontSeeTagTimer.start();
  
      rotController.setSetpoint(0);
      rotController.setTolerance(1);
  
      xController.setSetpoint(0);
      xController.setTolerance(0.02);
  
      yController.setSetpoint(isRightScore ? offset : 0);// if right score, setpoint is 0, else -0.1
      yController.setTolerance(0.02);
  
      tagID = LimelightHelpers.getFiducialID("limelight-intake");
    }
  
    @Override
    public void execute() {
      if (LimelightHelpers.getTV("limelight-intake") && LimelightHelpers.getFiducialID("limelight-intake") == tagID) {
        this.dontSeeTagTimer.reset();
  
        double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-intake");
        SmartDashboard.putNumber("x", postions[2]);
  
        double xSpeed = xController.calculate(postions[2]);
        SmartDashboard.putNumber("xspeed", xSpeed);
        double ySpeed = -yController.calculate(postions[0]);
        double rotValue = -rotController.calculate(postions[4]);
  
// drive!
        drivebase.setControl(m_drive
               // .withVelocityX(xSpeed) // Drive forward with negative Y(forward)
                .withVelocityY(ySpeed) // Drive left with negative X (left)
               // .withRotationalRate(rotValue)
               );


  
        if (!rotController.atSetpoint() ||
            !yController.atSetpoint() ||
            !xController.atSetpoint()) {
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
      // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
      return this.dontSeeTagTimer.hasElapsed(1) ||
          stopTimer.hasElapsed(0.3);
    }
  }