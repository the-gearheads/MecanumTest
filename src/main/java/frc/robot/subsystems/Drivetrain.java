package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArcadeDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

import java.util.HashMap;

public class Drivetrain extends SubsystemBase {

    private final WPI_TalonFX frontLMotor = new WPI_TalonFX(FMOTOR_LEFT);
    private final WPI_TalonFX rearLMotor = new WPI_TalonFX(BMOTOR_LEFT);
    private final WPI_TalonFX frontRMotor = new WPI_TalonFX(FMOTOR_RIGHT);
    private final WPI_TalonFX rearRMotor = new WPI_TalonFX(BMOTOR_RIGHT);
    private final SimpleMotorFeedforward feedFoward = new SimpleMotorFeedforward(FF_KS, FF_KV, FF_KA);
    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(FMOTOR_LEFT_POS, FMOTOR_RIGHT_POS,
            BMOTOR_LEFT_POS, BMOTOR_RIGHT_POS);
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final MecanumDriveOdometry odometry;

    public Drivetrain() {
        frontLMotor.configFactoryDefault();
        frontRMotor.configFactoryDefault();
        rearLMotor.configFactoryDefault();
        rearRMotor.configFactoryDefault();

        frontRMotor.setInverted(true);
        rearRMotor.setInverted(true);

        frontLMotor.setNeutralMode(NeutralMode.Brake);
        frontRMotor.setNeutralMode(NeutralMode.Brake);
        rearLMotor.setNeutralMode(NeutralMode.Brake);
        rearRMotor.setNeutralMode(NeutralMode.Brake);

        gyro.reset();
        odometry = new MecanumDriveOdometry(kinematics, gyro.getRotation2d());
        setDefaultCommand(new ArcadeDrive(this));
    }

    public void zeroEncoders() {
        frontLMotor.setSelectedSensorPosition(0);
        frontRMotor.setSelectedSensorPosition(0);
        rearLMotor.setSelectedSensorPosition(0);
        rearRMotor.setSelectedSensorPosition(0);
        gyro.zeroYaw();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPosition(Pose2d pose) {
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    private double nativeToMeters(double nativeUnits) {
        double rotations = nativeUnits / TALON_UNITS_PER_ROTATION; // convert native units to rotations
        double wheelRotations = rotations / SHAFT_TO_WHEEL_GEAR_RATIO; // convert rotations of motor shaft to rotations
                                                                       // of wheel
        double linearDisplacement = wheelRotations * WHEEL_CIRCUMFERENCE; // convert wheel rotations to linear
                                                                          // displacement
        return linearDisplacement;
    }

    public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, getPose().getRotation());
        drive(speeds);
    }

    /* Converts native velocity units to m/s */
    private double nativeVelocityToMeters(double velocity) {
        return nativeToMeters(velocity) * 10; // converts m/100ms to m/s
    }

    @Override
    public void periodic() {

        odometry.update(gyro.getRotation2d(), getWheelSpeeds());
    }

    public void drive(ChassisSpeeds speeds) {
        SmartDashboard.putNumber("rotSpd", speeds.omegaRadiansPerSecond);
        MecanumDriveWheelSpeeds whlSpeeds = kinematics.toWheelSpeeds(speeds);
        setSpeeds(whlSpeeds);
    }

    public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
        SmartDashboard.putNumber("fl", speeds.frontLeftMetersPerSecond);
        SmartDashboard.putNumber("fr", speeds.frontRightMetersPerSecond);
        SmartDashboard.putNumber("rl", speeds.rearLeftMetersPerSecond);
        SmartDashboard.putNumber("rr", speeds.rearRightMetersPerSecond);
        // something like // something like // something like thats something like:
        double flVoltage = feedFoward.calculate(speeds.frontLeftMetersPerSecond);
        double frVoltage = feedFoward.calculate(speeds.frontRightMetersPerSecond);
        double rlVoltage = feedFoward.calculate(speeds.rearLeftMetersPerSecond);
        double rrVoltage = feedFoward.calculate(speeds.rearRightMetersPerSecond);
        SmartDashboard.putNumber("flV", flVoltage);
        SmartDashboard.putNumber("frV", frVoltage);
        SmartDashboard.putNumber("rlV", rlVoltage);
        SmartDashboard.putNumber("rrV", rrVoltage);
        setVoltages(flVoltage, frVoltage, rlVoltage, rrVoltage);
    }

    public void setVoltages(double fLeftVoltage, double fRightVoltage, double rLeftVoltage, double rRightVoltage) {
        frontLMotor.setVoltage(MathUtil.clamp(fLeftVoltage, -12, 12));
        frontRMotor.setVoltage(MathUtil.clamp(fRightVoltage, -12, 12));
        rearRMotor.setVoltage(MathUtil.clamp(rRightVoltage, -1, 12));
        rearLMotor.setVoltage(MathUtil.clamp(rLeftVoltage, -12, 12));
    }

    // Assuming this method is part of a drivetrain subsystem that provides the
    // necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetPosition(traj.getInitialHolonomicPose());
                    }
                }),
                new PPMecanumControllerCommand(
                        traj,
                        this.odometry::getPoseMeters, // Pose supplier
                        this.kinematics, // MecanumDriveKinematics
                        new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0
                                                    // will only use feedforwards.
                        new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving
                                                    // them 0 will only use feedforwards.
                        3.0, // Max wheel velocity meters per second
                        this::setSpeeds, // MecanumDriveWheelSpeeds consumer
                        eventMap, // This argument is optional if you don't use event markers
                        this // Requires this drive subsystem
                ));
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        double fl = nativeVelocityToMeters(frontLMotor.getSelectedSensorVelocity());
        double fr = nativeVelocityToMeters(frontRMotor.getSelectedSensorVelocity());
        double rl = nativeVelocityToMeters(rearLMotor.getSelectedSensorVelocity());
        double rr = nativeVelocityToMeters(rearRMotor.getSelectedSensorVelocity());
        return new MecanumDriveWheelSpeeds(fl, fr, rl, rr);
    }
}