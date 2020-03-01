package com.team1806.frc2020.auto.actions;

public class LambdaAction implements Action {

    VoidInterace mF;

    public LambdaAction(VoidInterace f) {
        this.mF = f;
    }

    @Override
    public void start() {
        mF.f();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }

    public interface VoidInterace {
        void f();
    }
}