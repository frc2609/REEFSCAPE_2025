package frc.robot.commands.Align;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class PIDFineAlign extends Command {
    private PIDController xController, yController, rotController;
    private boolean isLeftScore;
    private static Timer dontSeeTagTimer, stopTimer;
    private CommandSwerveDrivetrain drivebase;
    private static  double tagID = -1;
    private SwerveRequest.RobotCentric m_drive = new SwerveRequest.RobotCentric();
    public static Trigger aligned = new Trigger(() -> stopTimer.hasElapsed(0.3) && tagID != -1);
    private String limelightName = "limelight-intake";
    SwerveRequest.RobotCentric rocDrive = new SwerveRequest.RobotCentric().withVelocityY(-1);

    public PIDFineAlign(boolean isLeftScore, CommandSwerveDrivetrain drivebase) {
      xController = new PIDController(3, 2, 0.0000);  // Vertical movement
      yController = new PIDController(3, 0.75, 0.0000);  // Horitontal movement
      rotController = new PIDController(0.1, 0.0001, 0.00);  // Rotation

      xController.setIZone(0.3);
      yController.setIZone(0.3);
      rotController.setIZone(2.0);

      this.isLeftScore = isLeftScore;
      this.drivebase = drivebase;
      addRequirements(drivebase);
    }
  
    @Override
    public void initialize() {
      PIDFineAlign.stopTimer = new Timer();
      PIDFineAlign.stopTimer.start();
      PIDFineAlign.dontSeeTagTimer = new Timer();
      PIDFineAlign.dontSeeTagTimer.start();

      // Left set points
      double Xsetpoint = -0.25;
      double Ysetpoint = -0.45;
      double rotSetPoint = 0;     ;

      // Right set points
      if (isLeftScore == false){
        limelightName = "limelight";
        Xsetpoint = -0.25;
        Ysetpoint = -0.12;
        rotSetPoint = 0;
      }
      if(isLeftScore && LimelightHelpers.getFiducialID(limelightName) == -1){
        drivebase.applyRequest(() -> rocDrive);
      }
      
      xController.setSetpoint(Xsetpoint);
      xController.setTolerance(0.02);
    
      yController.setSetpoint(Ysetpoint);
      yController.setTolerance(0.02);//0.02

      rotController.setSetpoint(rotSetPoint);
      rotController.setTolerance(1.0);
  
      tagID = LimelightHelpers.getFiducialID(limelightName);
    }

    @Override
    public void execute() {
      System.out.println("fine aligning");
      if (LimelightHelpers.getTV(limelightName) && LimelightHelpers.getFiducialID(limelightName) == tagID) {
        PIDFineAlign.dontSeeTagTimer.reset();
  
        double[] postions = LimelightHelpers.getBotPose_TargetSpace(limelightName);
        
        if (xController.getError() < 0.5) { 
          xController.setP(2.0);
        } else {
          xController.setP(4.0);
        }

        if (yController.getError() < 0.5) { 
          yController.setP(2.0);
        } else {
          yController.setP(4.0);
        }

        double xSpeed = xController.calculate(postions[2]);
        double ySpeed = -yController.calculate(postions[0]);
        double rotValue = -rotController.calculate(postions[4]);
        SmartDashboard.putNumber("error", yController.getError());

        // drive!
        drivebase.setControl(m_drive
           .withVelocityX(xSpeed) // Drive forward with negative Y(forward)
           .withVelocityY(ySpeed) // Drive left with negative X (left)
           .withRotationalRate(rotValue)
        );
  
        if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
          stopTimer.reset();
          }
        }

      else {
        drivebase.setControl(m_drive
                .withVelocityX(0) // Drive forward with negative Y(forward)
                .withVelocityY(0) // Drive left with negative X (left)
                .withRotationalRate(0));
      }
  
      SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }
  
    @Override
    public void end(boolean interrupted) {
      System.out.println("fine aligning done");
        drivebase.setControl(m_drive
        .withVelocityX(0) // Drive forward with negative Y(forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0));
    }
  
    @Override
    public boolean isFinished() {
      // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
      return PIDFineAlign.dontSeeTagTimer.hasElapsed(1) ||
          stopTimer.hasElapsed(0.3);
    }
  }