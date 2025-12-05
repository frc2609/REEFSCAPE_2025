package frc.robot.commands.Align;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers;

public class PIDFineAlignPrac extends Command{
    private double tA;
    private final CommandSwerveDrivetrain drivebase;
    private SwerveRequest.RobotCentric m_drive = new SwerveRequest.RobotCentric();
    public PIDFineAlignPrac(CommandSwerveDrivetrain drivebase) {

        this.drivebase = drivebase;
        addRequirements(drivebase);
        
    }
  
    @Override
    public void initialize() {
        System.out.println("Fine aligning starting");
    }

    @Override
    public void execute() {
        tA = LimelightHelpers.getTA("limelight");
        System.out.println(tA);
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
        return false;
    }
}
