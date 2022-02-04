#ifndef _SPEAKER_H_
#define _SPEAKER_H_

#include "defines.h"
#include <Arduino.h>
#include "HAL_ESP32.h"
#include "AXP192.h"

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#include "esp32-hal-dac.h"
#ifdef __cplusplus
}
#endif /* __cplusplus */

class SPEAKER
{
public:
    SPEAKER(AXP192 *axp);

    void begin();
    void end();
    void mute();
    void tone(uint16_t frequency);
    void tone(uint16_t frequency, uint32_t duration);
    void beep();
    void setBeep(uint16_t frequency, uint16_t duration);
    void update();

    void write(uint8_t value);
    void setVolume(uint8_t volume);
    void playMusic(const uint8_t *music_data, uint16_t sample_rate);

private:
    bool InitI2SSpeakOrMic(int mode);
    void DingDong(void);

    AXP192 *axp;
    uint32_t _count;
    uint8_t _volume;
    uint16_t _beep_duration;
    uint16_t _beep_freq;
    bool _begun;
    bool speaker_on;
};
#endif