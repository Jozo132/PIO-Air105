#ifndef AIR105_CLOCK_CONFIG_H
#define AIR105_CLOCK_CONFIG_H

#define AIR105_STARTUP_CPU_CLOCK_HZ 204000000UL

#ifndef F_CPU
#error "F_CPU must be provided by the AIR105 board configuration"
#elif F_CPU != AIR105_STARTUP_CPU_CLOCK_HZ
#error "AIR105 startup currently supports only F_CPU=204000000 Hz"
#endif

#endif