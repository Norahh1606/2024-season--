package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Swerve extends SubsystemBase {
    private final Joystick willController = new Joystick(0);
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID);
    public final Constraints translationConstraints = new Constraints(10, 10);
    public final PIDController xController = new PIDController(4, 0.0, 0.0);
    public final PIDController yController = new PIDController(4, 0.0, 0.0);
    public final PIDController rotaController = new PIDController(5, 0, 0);
    public final SimpleMotorFeedforward rotaFF = new SimpleMotorFeedforward(.23, 1);
    private final JoystickButton faceGrid = new JoystickButton(willController, XboxController.Button.kRightStick.value);
    private final JoystickButton dontFaceGrid = new JoystickButton(willController, XboxController.Button.kLeftStick.value);
    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.Swerve.trackWidth/ 2.0, Constants.Swerve.wheelBase / 2.0),
        new Translation2d(Constants.Swerve.trackWidth / 2.0, -Constants.Swerve.wheelBase / 2.0),
        new Translation2d(-Constants.Swerve.trackWidth / 2.0, Constants.Swerve.wheelBase / 2.0),
        new Translation2d(-Constants.Swerve.trackWidth / 2.0, -Constants.Swerve.wheelBase / 2.0)
    );

    public final SwerveModulePosition[] modulepositions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

   
   
    public Swerve() {

        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();
        rotaController.enableContinuousInput(0, 2*Math.PI);
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        

        };
        
    
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(kinematics, getYaw(), modulepositions);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
     ;
            SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotaFF.calculate(rotation), 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotaFF.calculate(rotation))
                                );
        setModuleStates(swerveModuleStates);
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
        /*
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(desiredStates);
        SmartDashboard.putNumber("SpeedX", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("SpeedY", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("SpeedR", chassisSpeeds.omegaRadiansPerSecond);*/
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble() + 360) : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }
/* 
    public Rotation2d getPitch() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getPitch()) : Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getRoll() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getRoll()) : Rotation2d.fromDegrees(gyro.getRoll());
    }
    */
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }



    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());

        SmartDashboard.putNumber("X Position", getPose().getX());
        SmartDashboard.putNumber("Y position", getPose().getY());
        //SmartDashboard.getNumber("Yaw", gyro.getYaw());
        //SmartDashboard.getNumber("Pitch", gyro.getPitch());
        //SmartDashboard.getNumber("Roll", gyro.getRoll());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}