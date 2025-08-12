package org.firstinspires.ftc.teamcode.controllers;

public class WaitTask extends TimedTask {

    public WaitTask(long duration) {
        super(
                duration,
                new TimedTaskListener() {
                    @Override
                    public void execute() {
                        // do nothing
                    }
                });
    }
}
