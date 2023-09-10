package org.firstinspires.ftc.teamcode.drive.localizer;

import org.firstinspires.ftc.teamcode.drive.geometry.Pose;

public interface Localizer {

    void periodic();

    Pose getPos();

    void setPos(Pose pose);
}
