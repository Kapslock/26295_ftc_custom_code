package org.nknsd.teamcode.controlSchemes.abstracts;

import org.nknsd.teamcode.frameworks.NKNControlScheme;

import java.util.concurrent.Callable;

// NEEDS CONSTRUCTION
public abstract class ShaiHuludControlScheme extends NKNControlScheme {

    public abstract Callable<Boolean> specimenGrab();

    public abstract Callable<Boolean> specimenRelease();

    public abstract Callable<Boolean> specimenForward();

    public abstract Callable<Boolean> specimenBackwards();

    public abstract Callable<Boolean> specimenRaise();

    public abstract Callable<Boolean> specimenLower();
}

