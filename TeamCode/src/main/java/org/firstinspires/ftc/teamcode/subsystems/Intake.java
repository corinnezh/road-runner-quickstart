package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private CRServoImplEx intakeServo;

    private boolean isIntaking;
    private boolean isOuttaking;

    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServoImplEx.class, "intake");
        intakeServo.setDirection(CRServoImplEx.Direction.FORWARD);
    }

}
