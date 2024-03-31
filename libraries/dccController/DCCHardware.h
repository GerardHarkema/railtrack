#ifndef __DCCHARDWARE_H__
#define __DCCHARDWARE_H__


#ifdef __cplusplus
extern "C"
{
#endif

int setup_DCC_waveform_generator(void);
void DCC_waveform_generation_hasshin(void);
int enableTrackPower();
int disableTrackPower();

#ifdef __cplusplus
}
#endif

#endif //__DCCHARDWARE_H__