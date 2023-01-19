package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class ControlHub {
    LynxDcMotorController hub;
    int[] bulkDataCache;

    public ControlHub(HardwareMap hardwareMap, LynxDcMotorController hub) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.hub = hub;
        bulkDataCache = this.internalRefreshBulkData();
    }

    private int[] internalRefreshBulkData() {
        int[] bulkData = new int[3];

        for (int i=0; i<3; i++) {
            bulkData[i] = hub.getMotorCurrentPosition(i);
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
