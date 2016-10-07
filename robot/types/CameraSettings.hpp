#pragma once

struct CameraSettings {
    unsigned int hflip;
    unsigned int vflip;
    unsigned int brightness;
    unsigned int contrast;
    unsigned int saturation;
    unsigned int hue;
    unsigned int sharpness;
    unsigned int backlightCompensation;
    unsigned int exposure;
    unsigned int gain;
    int whiteBalance;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version) {
      ar & hflip;
      ar & vflip;
      ar & brightness;
      ar & contrast;
      ar & saturation;
      ar & hue;
      ar & sharpness;
      ar & backlightCompensation;
      ar & exposure;
      ar & gain;
      ar & whiteBalance;
    }
};


