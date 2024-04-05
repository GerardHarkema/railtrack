#ifndef __DCCHARDWARE_H__
#define __DCCHARDWARE_H__


class waveform_generator{
    private:
        static void waveform_generator_timer_isr();
    public:
        bool setup_DCC_waveform_generator(void);
        void EnableWaveformGenerator(void);
        bool enableTrackPower();
        bool disableTrackPower();
};


#endif //__DCCHARDWARE_H__