// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

    private final CANSparkMax indexerMotor = new CANSparkMax(Constants.Indexer.INDEXER_CAN, MotorType.kBrushed); //FIXME
    private final DigitalInput indexerBeamBreak = new DigitalInput(Constants.Indexer.INDEXER_BEAM_BREAK_DIO); 
    
    private boolean indexerBeamBreakValue;


  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {

  }

  public void FeedShooter () {
    indexerMotor.set(0.1);
  }

  public void IndexNote() {
    if (indexerBeamBreakValue = true) {
      indexerMotor.set(0.1);
    }
    else {
      indexerMotor.set(0);
    }
  }

  @Override
  public void periodic() {
    indexerBeamBreakValue = indexerBeamBreak.get();
    // This method will be called once per scheduler run
  }
}
