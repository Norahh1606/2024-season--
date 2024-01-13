package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class DynamicTeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private Limelight s_Limelight;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier slowSpeedSup;
    private BooleanSupplier slowTurnSup;

    public DynamicTeleopSwerve(Swerve s_Swerve, 
                               
                               Limelight s_Limelight, 
                               DoubleSupplier translationSup, 
                               DoubleSupplier strafeSup, 
                               DoubleSupplier rotationSup, 
                               BooleanSupplier robotCentricSup 
                              ) {
        this.s_Swerve = s_Swerve;
       
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowSpeedSup = slowSpeedSup;
        this.slowTurnSup = slowTurnSup;
        this.s_Limelight = s_Limelight;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;

       
        //double dynamicDriveSpeed = (100 - (Math.pow(8.3,-8) * armPosition * armPosition))/100; //old equation
        
        


            if (slowSpeedSup.getAsBoolean() == true){
                translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * Constants.Swerve.slowSpeedMultiplier;
                strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * Constants.Swerve.slowSpeedMultiplier;
                rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            
           
        /* Drive */
        if (slowTurnSup.getAsBoolean() == true){
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.SlowTurnMultiplier * Constants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true);
            }

        else if (slowTurnSup.getAsBoolean() == false){
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true);
        }
    }
}
}