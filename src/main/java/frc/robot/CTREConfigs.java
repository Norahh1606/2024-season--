package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

/* 
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
*/
public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(Constants.Swerve.angleEnableCurrentLimit)
            .withSupplyCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
            .withSupplyCurrentThreshold(Constants.Swerve.anglePeakCurrentLimit) 
            .withSupplyTimeThreshold(Constants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.Slot0.kV = Constants.Swerve.angleKV;
        swerveAngleFXConfig.withCurrentLimits(angleSupplyLimit);

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(Constants.Swerve.driveEnableCurrentLimit)
            .withSupplyCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit) 
            .withSupplyCurrentThreshold(Constants.Swerve.drivePeakCurrentLimit) 
            .withSupplyTimeThreshold(Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kV = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.withCurrentLimits(driveSupplyLimit);
        swerveDriveFXConfig.OpenLoopRamps = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps = Constants.Swerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        
        //swerveCanCoderConfig.configAbsoluteSensorRange = AbsoluteSensorRangeValue.[0To360]};
        //swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        //swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        //swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    }
}