package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoPositions {
    public static Location smallForward = new Location(100, 0, 0);
    public static Location reset = new Location(-15,-25,0);
    public static Location middleZone = new Location(750, 75, 0);
    public static Location leftZone = new Location(900, -400, 0);
    public static Location rightZone = new Location(850, 950,0);

    public static Location coneZone = new Location(100, -175, 0);
    public static Location coneDrop = new Location(200, -175, 0);

}
