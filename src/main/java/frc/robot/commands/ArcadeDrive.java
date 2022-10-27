package frc.robot.commands;


import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;

/** An example command that uses an example subsystem. */
public class ArcadeDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private final XboxController controller = new XboxController(00);


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(Drivetrain subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var vx = -controller.getLeftY(); 
    var vy = -controller.getLeftX();
    var rot = -controller.getRightX();
    var RightTrigger = controller.getRightTriggerAxis()*TURBO_SPEED;
    var LeftTrigger = controller.getRightTriggerAxis()*TURBO_ROT_SPEED; // you cant put the entire calculation there, it wouldnt work or make any sense
    // its alomost 
    SmartDashboard.putNumber("vx", vx);
    SmartDashboard.putNumber("vy", vy);
    SmartDashboard.putNumber("rot", rot);
    SmartDashboard.putNumber("RightTrigger", RightTrigger);
    SmartDashboard.putNumber("LeftTrigger", LeftTrigger);
    m_subsystem.driveFieldRelative(new ChassisSpeeds(vx * DRIVE_SPEED * (RightTrigger+1), vy * DRIVE_SPEED* (RightTrigger+1), rot * ROTATE_SPEED * (LeftTrigger+1)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
