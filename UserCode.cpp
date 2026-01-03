#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
   gcs().send_text(MAV_SEVERITY_INFO,
                    "Servo9 RCIN9 gating enabled (100m)");
    //once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    uint16_t rc8_pwm = hal.rcin->read(7);
    #define RELAY_LED1 0   
#define RELAY_LED2 1 

    static uint32_t last_msg_ms = 0;
    const uint32_t now = AP_HAL::millis();
    float dist_m = 0;
    bool position_ok = false;
  
    // ---- HOME / POSITION / DISTANCE (real meters) ----
    if (ahrs.home_is_set()) {
        Vector2f ne_from_home;
        position_ok = ahrs.get_relative_position_NE_home(ne_from_home);
        if (position_ok) {
            dist_m = ne_from_home.length();
        }
    }

    bool safe_condition =
        (!ahrs.home_is_set() ||
         !position_ok ||
         dist_m < 200.0f ||
         !motors->armed());

    if (safe_condition) {
        SRV_Channels::set_output_pwm_chan(7, 1000);  // Safe actuator
         if (AP::relay() != nullptr) 
            {
                AP::relay()->off(RELAY_LED1);
                AP::relay()->off(RELAY_LED2);
            }
        if (now - last_msg_ms > 3500) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "SAFE: dist=%.1f trig=%d",
                            (double)dist_m, 30);
            last_msg_ms = now;
        }}
    else {
        if (rc8_pwm >= 800 && rc8_pwm <= 2200) {
            SRV_Channels::set_output_pwm_chan(7, rc8_pwm);
            
            uint16_t rc9_pwm = hal.rcin->read(8);
           // uint16_t rc10_pwm = hal.rcin->read(9);
             if (AP::relay() != nullptr) {
            if (rc9_pwm>800 && rc9_pwm<1300)
            {
                AP::relay()->on(RELAY_LED1);
                AP::relay()->off(RELAY_LED2);
            } 
            else if (rc9_pwm>1300 && rc9_pwm<1600)
            {
                AP::relay()->off(RELAY_LED1);
                AP::relay()->off(RELAY_LED2);
            }
            else if(rc9_pwm>1600 && rc9_pwm<2100)
            {
                AP::relay()->off(RELAY_LED1);
                AP::relay()->on(RELAY_LED2); 
            }}
             if (now - last_msg_ms > 3500) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "READY: dist=%.1f",
                            (double)dist_m);   
            last_msg_ms = now;
        }
           
}}
return;
    }
#endif


#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
 //static bool state = false;

   // uint16_t pwm = state ? 1900 : 1100;
   // state = !state;

   // SRV_Channels::set_output_pwm_chan(4, pwm);    // put your 1Hz code here
     //if (state) {
     //   gcs().send_text(MAV_SEVERITY_INFO, "USER: SERVO5 ON (1900us)");
    //} else {
       // gcs().send_text(MAV_SEVERITY_INFO, "USER: SERVO5 OFF (1100us)");
    //}
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
