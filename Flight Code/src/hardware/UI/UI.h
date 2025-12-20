#pragma once

class UI {
private:
     void playSong(int[], int, int, int);
public:
    void initialize();
    bool getButton();
    void setTone(int, int);
    void stopTone();
    void setRed(int);
    void setBlue(int);
    void setGreen(int);
    void setColor(int, int, int);
    double measureVoltage();
    void playRandomSong(int, int);
};