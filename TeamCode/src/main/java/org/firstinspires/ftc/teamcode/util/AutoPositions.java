package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoPositions {
    public static NOdoLocation smallForward = new NOdoLocation(100, 0, 0);
    public static NOdoLocation reset = new NOdoLocation(-15,-25,0);
    public static NOdoLocation middleZone = new NOdoLocation(750, 75, 0);
    public static NOdoLocation leftZone = new NOdoLocation(900, -400, 0);
    public static NOdoLocation rightZone = new NOdoLocation(850, 950,0);

    public static NOdoLocation coneZone = new NOdoLocation(100, -175, 0);
    public static NOdoLocation coneDrop = new NOdoLocation(200, -175, 0);

}
