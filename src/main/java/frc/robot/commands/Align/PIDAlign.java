package frc.robot.commands.Align;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class PIDAlign extends Command {
    private PIDController xController, yController, rotController;
    private Timer dontSeeTagTimer, stopTimer;
    private CommandSwerveDrivetrain drivebase;
    private double tagID = -1;
    private SwerveRequest.RobotCentric m_drive = new SwerveRequest.RobotCentric();

    public PIDAlign( CommandSwerveDrivetrain drivebase) {
      xController = new PIDController(2.0, 0.00001, 0.0);  // Vertical movement
      yController = new PIDController(2.0, 0.00001, 0.0);  // Horitontal movement
      rotController = new PIDController(0.15, 0.0001, 0.0);  // Rotation

      xController.setIZone(0.15);
      yController.setIZone(0.15);
      rotController.setIZone(2.0);

      this.drivebase = drivebase;
      addRequirements(drivebase);
    }
  
    @Override
    public void initialize() {
      this.stopTimer = new Timer();
      this.stopTimer.start();
      this.dontSeeTagTimer = new Timer();
      this.dontSeeTagTimer.start();
  
      xController.setSetpoint(-0.3);
      xController.setTolerance(0.15);
  
      yController.setSetpoint(-0.3);
      yController.setTolerance(0.15);

      rotController.setSetpoint(0.0);
      rotController.setTolerance(8.0);
  
      tagID = LimelightHelpers.getFiducialID("limelight-intake");
    }

  
    @Override
    public void execute() {
      if (LimelightHelpers.getTV("limelight-intake") && LimelightHelpers.getFiducialID("limelight-intake") == tagID) {
        this.dontSeeTagTimer.reset();
  
        double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-intake");
        SmartDashboard.putNumber("x", postions[2]);
  
        if (xController.getError() < 1.0) { xController.setP(2.0);} else {xController.setP(4.0);}
        if (yController.getError() < 1.0) { yController.setP(2.0);} else {yController.setP(4.0);}

        double xSpeed = xController.calculate(postions[2]);
        SmartDashboard.putNumber("xspeed", xSpeed);
        double ySpeed = -yController.calculate(postions[0]);
        double rotValue = -rotController.calculate(postions[4]);
  
// drive!
        drivebase.setControl(m_drive
          .withVelocityX(xSpeed) // Drive forward with negative Y(forward)
          .withVelocityY(ySpeed) // Drive left with negative X (left)
          .withRotationalRate(rotValue)
        );
  
        if ( !rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
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