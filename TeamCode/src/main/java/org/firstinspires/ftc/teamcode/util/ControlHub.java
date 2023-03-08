package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class ControlHub {
    LynxDcMotorController hub;
    int[] bulkDataCache;
    int[] junkTicks = new int[4];

    public ControlHub(HardwareMap hardwareMap, LynxDcMotorController hub) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.hub = hub;
        bulkDataCache = this.internalRefreshBulkData();
    }

    public void setJunkTicks(int motor) {
        refreshBulkData();
        this.junkTicks[motor] = bulkDataCache[motor];
    }

    public void setJunkTicks(int motor, int junkTicks) {
        this.junkTicks[motor] = junkTicks;
    }

    private int[] internalRefreshBulkData() {
        int[] bulkData = new int[4];

        for (int i=0; i<4; i++) {
            bulkData[i] = hub.getMotorCurrentPosition(i) - junkTicks[i];
        }

        return bulkData;
    }

    public void refreshBulkData() {
        bulkDataCache = this.internalRefreshBulkData();
    }

    public int getEncoderTicks(int motor) {
        return bulkDataCache[motor];
    }
}