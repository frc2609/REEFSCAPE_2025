package frc.robot.commands.Align;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimelightHelpers;

public class AlignCommand extends Command {
    private final CommandSwerveDrivetrain m_Swerve;
    private final Limelight m_limelight;
    final double HEIGHT_OF_LIMELIGHT = 0.47; // meters
    final double HEIGHT_OF_TARGET = 0.16; // meters
    PIDController m_pidController = new PIDController(0.06, 0.0, 0.0);
    final double PITCH = 10;

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
   .withDeadband(4.73 * 0.1).withRotationalDeadband(2 * 0.1); // Add a 10% deadband




    public AlignCommand(CommandSwerveDrivetrain swerve, Limelight limelight, Pigeon2 pidgey) {
        m_Swerve = swerve;
        m_limelight = limelight;


        SmartDashboard.putNumber("kpRange", 0.03);
        SmartDashboard.putNumber("kpAim", 0.03);
        addRequirements(swerve, limelight);
    }



    private double limelightAimProportional(double m_kpAim) {
        // kP (constant of proportionality)
        // Determines the aggressiveness of the proportional control loop
        double kP = m_kpAim;
        // Get the "tx" value from the Limelight
        //double targetingAngularVelocity = (distanceX-goalDistance) * kP;
        double targetingAngularVelocity = m_limelight.get_tx() * kP;

        SmartDashboard.putNumber("limelightX: ", m_limelight.get_tx());

    
        // Convert to radians per second for the drivetrain
        targetingAngularVelocity *= -TunerConstants.kSpeedAt12Volts.magnitude();
    
        // Invert since tx is positive when the target is to the right of the crosshair
        // targetingAngularVelocity *= 1.0;
    
        return targetingAngularVelocity;
    }
    
    // Proportional ranging control with Limelight's "ty" value
    // Works best if the Limelight's mount height and target mount height are different.
    private double limelightRangeProportional(double m_kpRange) {
    
        double kP = m_kpRange;


        // Get the "ty" value from the Limelight
        // double targetingForwardSpeed = m_Vision.getTY() * kP;


        //double targetingForwardSpeed = (distance-goalDistance) * kP;
        double targetingForwardSpeed = m_limelight.get_ty() * kP;

        
    
        // Convert to meters per second for the dr[]\ivetrain
        targetingForwardSpeed *= -TunerConstants.kSpeedAt12Volts.magnitude();
    
        // Invert the direction for proper control
        // targetingForwardSpeed *= 1.0;
    
        return targetingForwardSpeed;
    }



    
    public void execute(){
        // double rot = limelightAimProportional();
        double kpRange = SmartDashboard.getNumber("kpRange", 0.03);
        double kpAim = SmartDashboard.getNumber("kpAim", 0.05);
        LimelightHelpers.setPipelineIndex("limelight-intake", 2);
        // Store the ID of the AprilTag the Limelight is seeing
         double xSpeed = limelightRangeProportional(kpRange); 
         //use this for the aming if the april tag is able to still see the target
        System.out.println(xSpeed);

        m_Swerve.setControl(
            m_driveRequest
                .withVelocityX(limelightAimProportional(-0.1))  
                .withVelocityY(limelightRangeProportional(kpAim))
        );


    }

    public void end(boolean interrupted){
        LimelightHelpers.setPipelineIndex("limelight-intake", 1);
    }
}

