package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_topMotor = new SparkMax(IntakeConstants.kIntakeTopMotorID, MotorType.kBrushless);
    private final SparkMax m_bottomMotor = new SparkFMax(IntakeConstants.kIntakeBottomMotorID, MotorType.kBrushless);

    public void Intake(boolean input){
        int speed = input ? 1 : 0;
        m_topMotor.set(IntakeConstants.kIntakeSpeed*speed);
        m_bottomMotor.set(-IntakeConstants.kIntakeSpeed*speed);
    }
}
