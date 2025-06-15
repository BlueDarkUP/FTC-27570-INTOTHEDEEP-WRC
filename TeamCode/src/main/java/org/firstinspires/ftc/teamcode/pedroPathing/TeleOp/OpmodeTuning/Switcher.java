package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.OpmodeTuning;

public class Switcher {
    public boolean LastFlag;
    public boolean Flag = false;
    public void Switch(){
        if(Flag){
            Flag=false;
        }
        Flag=true;
    }
    public void RecordLF(boolean lastFlag){
        LastFlag = lastFlag;
    }
}
