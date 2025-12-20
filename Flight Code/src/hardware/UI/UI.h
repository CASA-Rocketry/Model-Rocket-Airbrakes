#pragma once

class UI {
public:
    static void initialize();
    static bool getButton();
    static void setTone(int, int);
    static void stopTone();
    static void setRed(int);
    static void setBlue(int);
    static void setGreen(int);
    static void setColor(int, int, int);
    static double measureVoltage();
};