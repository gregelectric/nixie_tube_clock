;****************************************************************************************
;* Nixie Clock 2
;*
;* Authors: Greg Powell, Larry Phillips, Steve Ansell and Steven Pridie 
;* 
;* Revision History:
;*  
;* V1.00 19 July 2004 - started with nixie gizmo code
;* V1.01 19 July 2004 - clock out 64 bits to shift registers
;* V1.03 20 July 2004 - changed element resolution down to 32 bits (increase interrupt to 156us)
;*                    - successfully clocked out 64 bits during 156us interrupt
;* V1.04 21 July 2004 - brightness control of all 64 outputs
;*                    - changed PWM resolution to 5 bit
;* V1.06 19 Aug  2004 - mapped output bits to respective elements
;* V1.07 20 Aug  2004 - add RTC code
;* V1.08 21 Aug  2004 - lookup table to unravel digit order in RAM
;* V1.11 22 Aug  2004 - displays seconds, minutes and hours
;* V1.12 22 Aug  2004 - handle 12/24 hour display in print_hours
;* V1.13 22 Aug  2004 - add IR remote control
;* V1.14 22 Aug  2004 - set time feature
;* V1.16 22 Aug  2004 - display set time digits as they are selected
;* V1.17 23 Aug  2004 - moved IR command parsing to main loop
;* V1.18 23 Aug  2004 - implemented ranges for set time to prevent invalid time set
;* V1.19 24 Aug  2004 - implemented date display and set
;* V1.21 25 Aug  2004 - implemented one button mode, menu + mode button for set date or time
;*                    - implemented blinking colon while in set time or date
;* V1.21 26 Aug  2004 - added TRIG input to test, optimize code
;* V1.23 27 Aug  2004 - optimize code, fixed menu timer bug
;* V1.26 28 Aug  2004 - fixed bug where digit 9 would be blank at startup
;* V1.28 28 Aug  2004 - added alarm function
;* V1.30 29 Aug  2004 - motion trigger to extend tube life (clock on for 1 hr after motion detected) 
;*                      (in place of alarm fucntion - no more code space!!!)
;* V1.31  1 Sept 2004 - motion sensor is normally closed, inverting input logic
;* V1.32  3 Sept 2004 - edge triggered motion sensor input, will work for NO or NC relay inputs
;* V1.33  4 Sept 2004 - fixed display off while setting time
;* V1.34  4 Sept 2004 - fixed power-up bug
;* V1.35  5 Sept 2004 - removed date function to make space for digit fading
;*                      added test function to test hardware
;* V1.36  5 Sept 2004 - switch to 12hr view
;* V1.39  6 Sept 2004 - 12hr and 24hr mode
;* V1.40  7 Sept 2004 - read 12/24hr mode from RTC, remember mode on power-up
;* V1.41  9 Sept 2004 - 12/24 hr mode toggle with one button press
;* V1.42 26 Sept 2004 - eliminate display flicker during IR input
;* V1.43 27 Sept 2004 - optimize 12/24hr mode change code with add_one_hr subroutine
;* V1.44 27 Sept 2004 - add leading zero blanking in 12hr mode
;* V1.45 28 Sept 2004 - add dimming of elements feature
;* V1.46  1 Oct  2004 - if POWER turned on by remote then alarm POWER off is ignored until POWER is turned
;*                      off using the remote - thus external trigger can be disabled
;* V1.47  5 Oct  2004 - fixed display update when changing bringtness level
;* V1.48  5 Oct  2004 - dimm all digits during set time, brighten digits as they are set
;* V1.49  6 Oct  2004 - fix bug, during set time if digits don't change should still brighten
;* V1.50  6 Oct  2004 - all constants capitalized for readability
;* V1.51  6 Oct  2004 - blank all digits when setting time, turn on as each digit is set

;* V2.00 26 May  2005 - ported code from AT90S2313 to ATmega8515
;* V2.01  5 Sept 2005 - fix startup bug (all elements turn on)
;* V2.02 16 Sept 2005 - added auto dimming for nighttime
;* V2.03 18 Sept 2005 - fix bug, no longer analog comparator interrupt (avoid interrupting RAM access and corruption)
;* V2.04  7 Oct  2005 - de-sputter?
;* V2.05  8 Oct  2005 - fixed colons not all turning on/off
;* V2.06  8 Oct  2005 - enter program mode using IR remote
;* V2.07  8 Oct  2005 - add support for temperature sensor - unfinished
;* V2.08 16 Oct  2005 - 3 level ambient brightness detect
;* V2.09 23 Oct  2005 - moved PWM to main loop, fading done with 10ms interrupt
;* V2.10 26 Oct  2005 - code cleanup
;* V2.11 27 Oct  2005 - bug fix - disable external int0 (external trigger input is polled)
;* V2.12 14 Jan  2006 - use remote to toggle relay on and off
;*                    - add beep feedback for remote button press
;*                    - changed IR delay down to 120ms to speed up button response
;*                    - conditional assembly for different board revisions
;* V2.13 15 Jan  2006 - re-implemented element test and desputter mode more effectively
;*                    - added full 12/24hr valid digit entry check
;*                    - added blink to colon when external trigger detected
;* V2.14 16 Jan  2006 - 12hr mode blinks colons
;* V2.15 22 Jan  2006 - store clock status (power, brightness setting and alarm time) to RAM on RTC
;*                    - fixed startup bug (colons and digits momentarily incorrect brightness)
;* V2.16 22 Jan  2006 - external trigger calles save_settings
;*                    - inverted AUTO_OFF_FLAG and prevented auto power down at power-up (unless external trigger)
;* V2.17 26 Jan  2006 - fixed external trigger on power-up (delay10ms for RC time const. and assume no input)
;* V2.18 27 Jan  2006 - two level ambient light detect
;*                    - colon cadence in set mode is 0.5sec, 1sec in 12hr display mode
;* V2.19 29 Jan  2006 - changed time input setting for set time/alarm/timer
;*                    - fixed bug in test element mode (leading zero blank in 12hr mode)
;*                    - fixed bug in set brightness (only update RAM if brightness changed - thanks to Rob Howland's help)
;* V2.20 31 Jan  2006 - add alarm setting/checking
;* V2.21  5 Feb  2006 - add alarm setting/checking/clearing
;* V2.22  7 Feb  2006 - PM indicator - top1 colon stays on for PM in 12hr mode (time and alarm display)
;*                    - enable/disable alarm and alarm enabled/disabled indicator
;* V2.23  8 Feb  2006 - no more blinky colons in 12hr mode (too confusing with indicators)
;*                    - re-aranged ir commands
;*                    - don't allow 12/24hr toggle during alarm time display
;* V2.24  8 Feb  2006 - backup alarm enabled in status register to RTC RAM (brightness now separate register)
;* V2.25 10 Feb  2006 - fixed bug in alarm (clear alarm_flag at the same place buzzer turned off) 
;*                    - else alarm may not go off like this morning and I was late for work!
;*                    - fixed bug where alarm could go off after executing show_alarm
;* V2.26 14 Feb  2006 - lowerd DIM to 3.15%
;* V2.27 17 Feb  2006 - used separate register for buzzer_timer (trig_in to RAM)
;* V2.29 24 Feb  2006 - fix hours ten flashing 0 briefly when set time in 24hr mode
;*                    - shut off buzzer and toggle relay after 1min
;* V2.31 31 Mar  2006 - fixed alarm not triggering bug (clear ALARM_FLAG in check_alarm_off)
;* V2.32  8 Apr  2006 - volume up/down changes brightness for both bright/dim ambient
;* V2.33  9 Apr  2006 - default values if RTC RAM corrupted by loss of power
;* V2.34 19 Apr  2006 - display FW version
;* V2.35 20 Apr  2006 - default values if RTC RAM corrupted by loss of power
;* V2.36 27 Oct  2006 - alarm enabled when set
;* V2.37 27 Oct  2006 - snooze feature
;*                    
;* Processor:         ATMEGA8515
;* Clock:             8MHz

;* Firmware Version
.equ  FW_VER_H     = 2     ; major version
.equ  FW_VER_L     = 37    ; minor version
 
;****************************************************************************************

.include "C:\Program Files\Atmel\AVR Tools\AvrAssembler2\Appnotes\m8515def.inc"
;.include "C:\My Documents\Appnotes\m8515def.inc"

;.equ  CLOCK2 = 1   ; compile for Nixie Clock 2 layout (different nixie element connections)
.equ  CLOCK3 = 1   ; compile for Nixie Clock 3 layout

;****************************************************************************************
;* Registers

; Lower registers cannont be used with immediates
.def  memdata        = r0       ; reserved for use with the LPM instruction
.def  zero           = r1
.def  sec_ten_pwm    = r2       ; brightness of digit
.def  minutes_pwm    = r3
.def  seconds_pwm    = r4
.def  min_ten_pwm    = r5
.def  hours_pwm      = r6
.def  hrs_ten_pwm    = r7
.def  buzzer_timer   = r13       ; used to time buzzer
.def  ir_byte_lo     = r8       ; NOTE that r8 is used in two places
.def  ir_byte_hi     = r9       ; ir_byte is only used in IR receive interupt
.def  ir_timer       = r10
.def  menu_timer     = r11      ; used to time-out menu button
.def  counter        = r12      ; used in IR receive interrupt
;.def  trig_in        = r13      ; state of the external trigger input (edge triggered)
.def  colon_timer    = r14      ; used to blink colons while setting date or time
.def  brightness     = r15      ; maximum brightness setting of elements (least significant 6 bits) 

; Upper registers can be used with immediates
.def  temp           = r16
.def  temp_2         = r17
.def  mode           = r18      ; modes are described by mode constants
.def  cnt1           = r19
.def  cnt2           = r20
.def  flags          = r21      ; each bit is defined by flag bits
.def  flags_2        = r22      ; more flag bits
.def  ir_command     = r23      ; command recieved by IR input
.def  pwmcounter     = r24
.def  fade_flags     = r25      ; inidicates when a digit is being faded
.def  ext_timer      = r26      ; (XL) timer for external trigger indicator
.def  rand_hi        = r27      ; (XH) used for random number generator 
.def  rand_lo        = r28      ; (YL) 
.def  digit_count    = r29      ; (YH)  used to count input digits while setting the time
;.def  ram_lo         = r30     ; (ZL)   used to point to RAM
;.def  ram_hi         = r31     ; (ZH)

;***********************************************************************************************
;PORTA bits
;.equ usused          = 0             ; Output - 
;.equ usused          = 1             ; Output - 
.equ BOOST_POWER     = 2             ; Output - boost regulator enable 
;.equ usused          = 3             ; Output - 
;.equ usused          = 4             ; Output - 
;.equ usused          = 5             ; Input - 
;.equ usused          = 6             ; Output - 
;.equ usused          = 7             ; Input -  
 
.equ PORTA_DIR  = 0b11111111
.equ PORTA_INIT = 0b00000100

;***********************************************************************************************
;PORTB bits
.equ  BUZZ           = 0            ; output - alarm tone or audio feedback
;.equ usused          = 1            ; Output - unused outputs
.equ PROGRAM         = 2            ; Input - enter program mode at power up
.equ LIGHT_SENSE     = 3            ; Input - light sensor input comparator
.equ RES1            = 4            ; Output - 
.equ RES2            = 5            ; Output - resistor pulldown (6K8)
.equ RES3            = 6            ; Output - 
.equ LED             = 7            ; Output - LED output
                          
.equ PORTB_DIR  = 0b11110011
.equ PORTB_INIT = 0b10000100

;***********************************************************************************************
;PORTC bits                              
;.equ usused          = 0            ; output - unused outputs
.equ  RTC_RESET      = 1            ; output - RTC data      
.equ  RTC_IO         = 2            ; output - RTC reset
.equ  RTC_CLOCK      = 3            ; output - RTC clock
.equ  TMP_CS         = 4            ; output - temp sensor chip select
.equ  STROBE         = 5            ; output - shift register strobe
.equ  DATA           = 6            ; output - shift register data
.equ  CLK            = 7            ; output - shift register clock

.equ PORTC_DIR  = 0b11111011
.equ PORTC_INIT = 0b00110000        ; strobe normally high

;***********************************************************************************************
;PORTD bits
;.equ  RXD            = 0            ; output - unused
;.equ  TXD            = 1            ; output - unused
.equ  TRIG           = 2            ; input - trigger input from motion sensor
.equ  IR_INPUT       = 3            ; input - IR control input
.equ  RELAY          = 4            ; ouput - relay
.equ  BUTTON1        = 5            ; input - button
.equ  BUTTON2        = 6            ; input - button
.equ  BUTTON3        = 7            ; input - button
 
.equ PORTD_DIR  = 0b00010000 
.equ PORTD_INIT = 0b11100100      ; POWER off at start, trig pullup enabled 


;****************************************************************************************
; flags - bit definitions
.equ  POWER_FLAG     = 0          ; set if LEDs enabled
.equ  AM_PM_FLAG     = 1          ; store am/pm setting when setting time/alarm in 12hr mode
.equ  MENU_FLAG      = 2          ; menu button pressed, time-out started
.equ  IR_FLAG        = 3          ; valid ir message received
.equ  MILITARY_FLAG  = 4          ; set for 24hr mode, clear for 12hr mode
.equ  AUTO_OFF_FLAG  = 5          ; if set then auto power down after 1hr
.equ  TRIGGER_FLAG   = 6          ; indicates an external trigger input
.equ  SNOOZE_FLAG    = 7          ; snooze button hit

; flags_2 - bit definitions
.equ  SET_TIME_FLAG  = 0          ; set if in set time mode
.equ  SET_ALARM_FLAG = 1          ; set if in set alarm mode
.equ  SET_TIMER_FLAG = 2          ; set if in set timer mode
.equ  ALARM_FLAG     = 3          ; set if ALARM time has occured
.equ  ALARM_EN_FLAG  = 4          ; set if ALARM is enabled
.equ  SET_FLAG       = 5          ; while entering digits for set time/alarm/timer
.equ  DISP_ALRM_FLAG = 6          ; used to display alarm setting for 2 seconds
.equ  ALARM_OFF_FLAG = 7          ; used to turn alarm off automatically

; digit fade flags
.equ  SECONDS_FLAG   = 0          ; seconds digit being faded
.equ  SEC_TEN_FLAG   = 1
.equ  MINUTES_FLAG   = 2
.equ  MIN_TEN_FLAG   = 3
.equ  HOURS_FLAG     = 4
.equ  HRS_TEN_FLAG   = 5
.equ  FADE_FLAG      = 6
.equ  COLONS_FLAG    = 7

;****************************************************************************************
;* Constants

; mode constants, the clock can only be in one mode at a time
.equ  CLOCK_MODE     = 0
.equ  ALARM_MODE     = 1            ; to be implemented 
.equ  TEMP_MODE      = 2            ; to be implemented
.equ  TIMER_MODE     = 3            ; to be implemented
.equ  DESPUTTER_MODE = 4            ; to be implemented

; ambient brightness level
.equ  AMB_BRIGHT     = 0            ; ambient is bright
.equ  AMB_MEDIUM     = 1            ; ambient is medium
.equ  AMB_DIM        = 2            ; ambient is dim

; PWM constants for brightness
.equ  BRIGHT         = 32           ; PWM dutycycle is 100%
.equ  MEDIUM         = 8            ; PWM dutycycle is 8 of 32, 25%
.equ  DIM            = 1            ; PWM dutycycle is 1 of 32, 3.15%

; IR remote constants
.equ  POWER          = 12           ; power button on IR remote control
.equ  VOLUME_UP      = 16
.equ  VOLUME_DOWN    = 17
.equ  MUTE           = 13
.equ  CHANNEL_UP     = 32
.equ  CHANNEL_DOWN   = 33
.equ  PREV_CHANNEL   = 34
.equ  ENTER          = 53
.equ  MENU           = 46

; RTC Commands          Bit meanings for register
;                       h = halt, t = tens, u = units
;                       M = Military/Civil, P = AM/PM (or t if mil)
;                       W = Write Protect
.equ  AM_PM_BIT       = 1
.equ  HR_12_BIT       = 3

.equ  WR_SEC         = $80   ; h t t t u u u u
.equ  RD_SEC         = $81   
.equ  WR_MIN         = $82   ; 0 t t t u u u u
.equ  RD_MIN         = $83
.equ  WR_HOUR        = $84   ; M 0 P t u u u u
.equ  RD_HOUR        = $85
.equ  WR_DATE        = $86   ; 0 0 t t u u u u
.equ  RD_DATE        = $87
.equ  WR_MONTH       = $88   ; 0 0 0 t u u u u
.equ  RD_MONTH       = $89
.equ  WR_DAY         = $8A   ; 0 0 0 0 0 u u u
.equ  RD_DAY         = $8B
.equ  WR_YEAR        = $8C   ; t t t t u u u u
.equ  RD_YEAR        = $8D
.equ  WR_CONTROL     = $8E   ; W 0 0 0 0 0 0 0
.equ  RD_CONTROL     = $8F
.equ  WR_TRICKLE     = $90   ; c c c c d d r r (c = enable, d = diode, r = resistor)
.equ  RD_TRICKLE     = $91
.equ  WR_BURST       = $BE   ; write a burst.
.equ  RD_BURST       = $BF
; start of user RAM
.equ  WR_SEC_ALARM   = $C0   ; h t t t u u u u 
.equ  RD_SEC_ALARM   = $C1
.equ  WR_MIN_ALARM   = $C2   ; 0 t t t u u u u
.equ  RD_MIN_ALARM   = $C3
.equ  WR_HOUR_ALARM  = $C4   ; M 0 P t u u u u
.equ  RD_HOUR_ALARM  = $C5
.equ  WR_STATUS      = $C6   ;  status bit definitions [  7  | 6   |  5  |  4  |  3  |  2  |  1  |  0  ]
.equ  RD_STATUS      = $C7   ;                         [     |     |     |     |     |ALRM |RELAY| PWR ]
.equ  WR_BRIGHT      = $C8   ;  element bright setting [  7  | 6   |  5  |  4  |  3  |  2  |  1  |  0  ]
.equ  RD_BRIGHT      = $C9   ;                         [     |     |    6 bit brightness setting       ]
.equ  WR_DIM         = $CA   ;  element dim setting    [  7  | 6   |  5  |  4  |  3  |  2  |  1  |  0  ]
.equ  RD_DIM         = $CB   ;                         [     |     |    6 bit brightness setting       ]

;****************************************************************************************
; SRAM

.dseg                               ; starting with $60

ram_start:

; the order and location of this ram must remain intact
element_ram_start:   
; PWM dutycycle for each element

.ifdef CLOCK2
.message "COMPILED FOR NIXIE CLOCK 2"

seconds_7:     .byte 1   ; bit  0  seconds element  7
seconds_6:     .byte 1   ; bit  1  seconds element  6
seconds_5:     .byte 1   ; bit  2  seconds element  5
seconds_4:     .byte 1   ; bit  3  seconds element  4
seconds_3:     .byte 1   ; bit  4  seconds element  3
seconds_8:     .byte 1   ; bit  5  seconds element  8
seconds_2:     .byte 1   ; bit  6  seconds element  2
seconds_1:     .byte 1   ; bit  7  seconds element  1
sec_tens_8:    .byte 1   ; bit  8  tens of seconds element 8
sec_tens_3:    .byte 1   ; bit  9  tens of seconds element 3
sec_tens_7:    .byte 1   ; bit 10  tens of seconds element 7 
sec_tens_6:    .byte 1   ; bit 11  tens of seconds element 6
sec_tens_2:    .byte 1   ; bit 12  tens of seconds element 2
sec_tens_1:    .byte 1   ; bit 13  tens of seconds element 1
seconds_0:     .byte 1   ; bit 14  seconds element  0
seconds_9:     .byte 1   ; bit 15  seconds element  9
col_ai:        .byte 1   ; bit 16  bottom colon - alarm enabled indicator
col_pm:        .byte 1   ; bit 17  top colon - PM indicator
minutes_1:     .byte 1   ; bit 18  minutes element  1
minutes_2:     .byte 1   ; bit 19  minutes element  2
sec_tens_0:    .byte 1   ; bit 20  tens of seconds element 0
sec_tens_9:    .byte 1   ; bit 21  tens of seconds element 9
sec_tens_4:    .byte 1   ; bit 22  tens of seconds element 4
sec_tens_5:    .byte 1   ; bit 23  tens of seconds element 5
minutes_5:     .byte 1   ; bit 24  minutes element  5
minutes_4:     .byte 1   ; bit 25  minutes element  4
minutes_9:     .byte 1   ; bit 26  minutes element  9
minutes_0:     .byte 1   ; bit 27  minutes element  0
minutes_6:     .byte 1   ; bit 28  minutes element  6
minutes_7:     .byte 1   ; bit 29  minutes element  7
minutes_3:     .byte 1   ; bit 30  minutes element  3
minutes_8:     .byte 1   ; bit 31  minutes element  8
min_tens_7:    .byte 1   ; bit 32  tens of minutes element 7 
min_tens_6:    .byte 1   ; bit 33  tens of minutes element 6
min_tens_5:    .byte 1   ; bit 34  tens of minutes element 5
min_tens_4:    .byte 1   ; bit 35  tens of minutes element 4
min_tens_3:    .byte 1   ; bit 36  tens of minutes element 3
min_tens_8:    .byte 1   ; bit 37  tens of minutes element 8
min_tens_2:    .byte 1   ; bit 38  tens of minutes element 2
min_tens_1:    .byte 1   ; bit 39  tens of minutes element 1
hours_1:       .byte 1   ; bit 40  hours element  1
hours_2:       .byte 1   ; bit 41  hours element  2
hours_8:       .byte 1   ; bit 42  hours element  8
hours_3:       .byte 1   ; bit 43  hours element  3
col_ext:       .byte 1   ; bit 44  top colon - external trigger indicator
colbot0:       .byte 1   ; bit 45  bottom colon
min_tens_0:    .byte 1   ; bit 46  tens of minutes element 0
min_tens_9:    .byte 1   ; bit 47  tens of minutes element 9
hours_9:       .byte 1   ; bit 48  hours element  9
hours_0:       .byte 1   ; bit 49  hours element  0
hours_tens_1:  .byte 1   ; bit 50  tens of hours element  1
hours_tens_2:  .byte 1   ; bit 51  tens of hours element  2
hours_4:       .byte 1   ; bit 52  hours element  4
hours_5:       .byte 1   ; bit 53  hours element  5
hours_6:       .byte 1   ; bit 54  hours element  6 
hours_7:       .byte 1   ; bit 55  hours element  7
hours_tens_5:  .byte 1   ; bit 56  tens of hours element  5
hours_tens_4:  .byte 1   ; bit 57  tens of hours element  4
hours_tens_9:  .byte 1   ; bit 58  tens of hours element  9
hours_tens_0:  .byte 1   ; bit 59  tens of hours element  0
hours_tens_6:  .byte 1   ; bit 60  tens of hours element  6
hours_tens_7:  .byte 1   ; bit 61  tens of hours element  7
hours_tens_3:  .byte 1   ; bit 62  tens of hours element  3
hours_tens_8:  .byte 1   ; bit 63  tens of hours element  8
blank:         .byte 1   ; bit 64  dummy RAM for leading zero blanking

.endif
.ifdef CLOCK3
.message "COMPILED FOR NIXIE CLOCK 3"

seconds_0:     .byte 1   ; bit  0  seconds element  0
seconds_1:     .byte 1   ; bit  1  seconds element  1
seconds_2:     .byte 1   ; bit  2  seconds element  2
seconds_3:     .byte 1   ; bit  3  seconds element  3
seconds_9:     .byte 1   ; bit  4 seconds element  9
seconds_8:     .byte 1   ; bit  5  seconds element  8
seconds_7:     .byte 1   ; bit  6  seconds element  7
seconds_6:     .byte 1   ; bit  7  seconds element  6
sec_tens_8:    .byte 1   ; bit  8  tens of seconds element 8
sec_tens_9:    .byte 1   ; bit  9  tens of seconds element 9
sec_tens_0:    .byte 1   ; bit 10  tens of seconds element 0
sec_tens_1:    .byte 1   ; bit 11  tens of seconds element 1
sec_tens_7:    .byte 1   ; bit 12  tens of seconds element 7 
sec_tens_6:    .byte 1   ; bit 13  tens of seconds element 6
seconds_5:     .byte 1   ; bit 14  seconds element  5
seconds_4:     .byte 1   ; bit 15  seconds element  4
col_ai:        .byte 1   ; bit 17  bottom colon - alarm enabled indicator
col_pm:        .byte 1   ; bit 16  top colon - PM indicator
minutes_6:     .byte 1   ; bit 18  minutes element  6
minutes_7:     .byte 1   ; bit 19  minutes element  7
sec_tens_5:    .byte 1   ; bit 20  tens of seconds element 5
sec_tens_4:    .byte 1   ; bit 21  tens of seconds element 4
sec_tens_3:    .byte 1   ; bit 22  tens of seconds element 3
sec_tens_2:    .byte 1   ; bit 23  tens of seconds element 2
minutes_2:     .byte 1   ; bit 24  minutes element  2
minutes_3:     .byte 1   ; bit 25  minutes element  3
minutes_4:     .byte 1   ; bit 26  minutes element  4
minutes_5:     .byte 1   ; bit 27  minutes element  5
minutes_1:     .byte 1   ; bit 28  minutes element  1
minutes_0:     .byte 1   ; bit 29  minutes element  0
minutes_9:     .byte 1   ; bit 30  minutes element  9
minutes_8:     .byte 1   ; bit 31  minutes element  8
min_tens_0:    .byte 1   ; bit 32  tens of minutes element 0
min_tens_1:    .byte 1   ; bit 33  tens of minutes element 1
min_tens_2:    .byte 1   ; bit 34  tens of minutes element 2
min_tens_3:    .byte 1   ; bit 35  tens of minutes element 3
min_tens_9:    .byte 1   ; bit 36  tens of minutes element 9
min_tens_8:    .byte 1   ; bit 37  tens of minutes element 8
min_tens_7:    .byte 1   ; bit 38  tens of minutes element 7 
min_tens_6:    .byte 1   ; bit 39  tens of minutes element 6
hours_6:       .byte 1   ; bit 40  hours element  6 
hours_7:       .byte 1   ; bit 41  hours element  7
hours_8:       .byte 1   ; bit 42  hours element  8
hours_9:       .byte 1   ; bit 43  hours element  9
col_ext:       .byte 1   ; bit 44  top colon
colbot0:       .byte 1   ; bit 45  bottom colon
min_tens_5:    .byte 1   ; bit 46  tens of minutes element 5
min_tens_4:    .byte 1   ; bit 47  tens of minutes element 4
hours_4:       .byte 1   ; bit 48  hours element  4 
hours_5:       .byte 1   ; bit 49  hours element  5
hours_tens_6:  .byte 1   ; bit 50  tens of hours element  6
hours_tens_7:  .byte 1   ; bit 51  tens of hours element  7
hours_3:       .byte 1   ; bit 52  hours element  3
hours_2:       .byte 1   ; bit 53  hours element  2
hours_1:       .byte 1   ; bit 54  hours element  1
hours_0:       .byte 1   ; bit 55  hours element  0
hours_tens_2:  .byte 1   ; bit 56  tens of hours element  2
hours_tens_3:  .byte 1   ; bit 57  tens of hours element  3
hours_tens_4:  .byte 1   ; bit 58  tens of hours element  4
hours_tens_5:  .byte 1   ; bit 59  tens of hours element  5
hours_tens_1:  .byte 1   ; bit 60  tens of hours element  1
hours_tens_0:  .byte 1   ; bit 61  tens of hours element  0
hours_tens_9:  .byte 1   ; bit 62  tens of hours element  9
hours_tens_8:  .byte 1   ; bit 63  tens of hours element  8
blank:         .byte 1   ; bit 64  dummy RAM for leading zero blanking

.endif
element_ram_end:

; the order of these bytes must remain the same
seconds:       .byte 1   ; current time
seconds_tens:  .byte 1
minutes:       .byte 1
minutes_tens:  .byte 1
hours:         .byte 1
hours_tens:    .byte 1
date:          .byte 1
date_ten:      .byte 1
month:         .byte 1
month_ten:     .byte 1
day:           .byte 1
day_ten:       .byte 1
year:          .byte 1
year_ten:      .byte 1
control_lo:    .byte 1   ; status of the control register low nybble (IE write protection)
control_hi:    .byte 1   ; high nybble

; used to track changes in display elements
cur_sec:       .byte 1   ; stores the current element that is on
cur_sec_ten:   .byte 1
cur_min:       .byte 1
cur_min_ten:   .byte 1
cur_hr:        .byte 1   
cur_hr_ten:    .byte 1   
last_sec:      .byte 1   ; stores the last element that was on
last_sec_ten:  .byte 1
last_min:      .byte 1
last_min_ten:  .byte 1
last_hr:       .byte 1
last_hr_ten:   .byte 1

auto_sec:     .byte 1   ; stores the external trigger time + 1hr for auto off feature
auto_sec_ten: .byte 1
auto_min:     .byte 1
auto_min_ten: .byte 1
auto_hrs:     .byte 1
auto_hrs_ten: .byte 1

alarm_sec:      .byte 1   ; local copy of alarm time (from RTC RAM)
alarm_sec_ten:  .byte 1
alarm_min:      .byte 1
alarm_mins_ten: .byte 1
alarm_hrs:      .byte 1
alarm_hrs_ten:  .byte 1

alarm_off_sec:      .byte 1   ; time to turn alarm off
alarm_off_sec_ten:  .byte 1
alarm_off_min:      .byte 1
alarm_off_mins_ten: .byte 1

snooze_sec:      .byte 1   ; snooze time, only seconds and minutes (snooze time is fixed)
snooze_sec_ten:  .byte 1
snooze_min:      .byte 1
snooze_mins_ten: .byte 1

trig_in:       .byte 1  ; trigger input

; PWM values for brightness
bright_ram:    .byte 1   ; PWM dutycycle setpoint for bright ambient light
dim_ram:       .byte 1   ; PWM dutycycle setpoint for dim ambient light

ram_end:

;****************************************************************************************
; Macros

.cseg

.macro   addi              ; There is no add immediate instruction, so we
   subi    @0, -@1         ; subtract the negative of an immediate value
.endm

.macro   power_on                 ; turn nixie power supply on
   cbi   porta, BOOST_POWER
.endm
.macro   power_off                ; turn nixie power supply off
   sbi   porta, BOOST_POWER
.endm

.macro   relay_on                 ; turn relay on
   sbi   portd, RELAY
.endm
.macro   relay_off                ; turn relay off
   cbi   portd, RELAY
.endm

.macro   led_on                   ; led relay on
   cbi   portb, LED
.endm
.macro   led_off                ; led relay off
   sbi   portb, LED
.endm

.macro   beep                     ; turn buzzer on for 10ms
   ldi   temp, 1
   mov   buzzer_timer, temp
   sbi   portb, BUZZ
.endm

.macro   buzzer_on                ; turn buzzer on
   sbi   portb, BUZZ
.endm
.macro   buzzer_off               ; turn buzzer off
   cbi   portb, BUZZ
.endm

.macro   colons_on                 
   sts   col_ext, brightness
   sts   colbot0, brightness
   sts   col_pm, brightness
   sts   col_ai, brightness
.endm

.macro   colons_off                
   sts   col_ext, zero
   sts   colbot0, zero
   sts   col_pm, zero
   sts   col_ai, zero
.endm

.macro   ex_trig_on                 
   sts   col_ext, brightness
.endm

.macro   ex_trig_off                
   sts   col_ext, zero
.endm

;****************************************************************************************
;* Interrupt Jump Table

.org 0
      rjmp  reset             ; ignite the tubes
.org  INT0addr                ; External Interrupt0 Vector Address     
      reti
.org  INT1addr                ; External Interrupt1 Vector Address
      rjmp  ir_receive        ; IR recieve int
.org  ICP1addr                ; Input Capture1 Interrupt Vector Address
      reti
.org  OC1Aaddr                ; Output Compare1A Interrupt Vector Address
      reti
.org  OC1Baddr                ; Output Compare1B Interrupt Vector Address
      reti
.org  OVF1addr                ; Overflow1 Interrupt Vector Address
      reti
.org  OVF0addr                ; Overflow0 Interrupt Vector Address
      rjmp  timer0_handler    ; IR recieve timer
.org  SPIaddr                 ; SPI Interrupt Vector Address
      reti
.org  URXCaddr                ; UART Receive Complete Interrupt Vector Address
      reti
.org  UDREaddr                ; UART Data Register Empty Interrupt Vector Address
      reti
.org  UTXCaddr                ; UART Transmit Complete Interrupt Vector Address
      reti
.org  ACIaddr                 ; Analog Comparator Interrupt Vector Address
      reti
.org  INT2addr                ; External Interrupt2 Vector Address
      reti
.org  OC0addr                 ; Output Compare0 Interrupt Vector Address
      reti
.org  ERDYaddr                ; EEPROM Interrupt Vector Address
      reti
;.org  SPMaddr                 ; SPM complete Interrupt Vector Address
;      reti
.org  SPMRaddr                ; SPM complete Interrupt Vector Address
      reti

;****************************************************************************************
;* Initialize Micro

reset:
   ldi    temp, low(RAMEND)   ; Initialize the Stack Pointer.
   out    spl, temp
   ldi    temp, high(RAMEND)
   out    sph, temp

   ldi   temp, PORTA_INIT     ;Initialize PORTD
   out   PORTA, temp
   ldi   temp, PORTA_DIR       
   out   DDRA, temp
   ldi   temp, PORTB_INIT     ;Initialize PORTB
   out   PORTB, temp          
   ldi   temp, PORTB_DIR
   out   DDRB, temp
   ldi   temp, PORTC_INIT     ;Initialize PORTD
   out   PORTC, temp
   ldi   temp, PORTC_DIR       
   out   DDRC, temp
   ldi   temp, PORTD_INIT     ;Initialize PORTD
   out   PORTD, temp
   ldi   temp, PORTD_DIR       
   out   DDRD, temp 

; timer 0 interrupt for IR receive 
   ldi   temp, (1 << CS02) | (1 << CS00)  ; CK/1024
   out   TCCR0, temp
   ldi   temp, (255 - 78)        ; 78 -> 10ms
   out   TCNT0, temp

; timer 1 interrupt for tube elements PWM
; element period:
; t = 1/32 * 1/100Hz = 312us
;
; t = 1/fclk * A * B
; fclk = 8E6
; A = 8
; B = 312

;   ldi   temp, $01
;   out   OCR1AH, temp
;   ldi   temp, $38                         ; 312us interrupt with CK/8
;   out   OCR1AL, temp  
;   ldi   temp, (1 << CS11) | (1 << WGM12)  ; CK/8, clear on compare match
;   out   TCCR1B, temp

   ldi   temp, (1 << TOIE0); | (1 << OCIE1A)   ; enable timer 0 overflow & timer 1 output compare interrupt 
   out   TIMSK, temp   

; external interrupts
   ldi   temp, (1 << INT1); | (1 << INT0)  ;enable ext. interrupt 1 and 0
   out   GIMSK, temp
   ldi   temp, (1 << ISC11)               ;set interrupt 1 for low level and 0 for falling edge
   out   MCUCR, temp

; analog comparator
   ldi   temp, (1<<ACBG)|(0<<ACD);|(1<<ACIS1)|(1<<ACIE); bandgap select, comparator on, int on falling edge, enable int
   out   ACSR, temp

; initialize RAM to zero
   clr   temp
   ldi   ZL, low(ram_start)
   ldi   ZH, high(ram_start)
ram_loop:
   st    Z+, temp
   cpi   ZH, high(ram_end)
   brne  ram_loop
   cpi   ZL, low(ram_end)
   brne  ram_loop          

; initialize "last" element registers to 1, "current" element registers are 0
; print_time requires last and current to be different before updating a digit
; if last and current were left the same, ie. 0 then 0's would be blank at startup
   ldi   temp, 10
   ldi   ZH, high(last_sec)
   ldi   ZL, low(last_sec)
   st    Z+, temp
   st    Z+, temp
   st    Z+, temp
   st    Z+, temp
   st    Z+, temp
   st    Z, temp 

   clr   zero
   clr   mode
   clr   flags
   clr   flags_2
   clr   fade_flags
   clr   menu_timer
   clr   ir_timer
   clr   colon_timer
   ldi   temp, 32
   mov   brightness, temp
   ldi   pwmcounter, 1
   ldi   temp, (1 << TRIG)              ; mask for trigger input
   sts   trig_in, temp                  ; assume no external contact close on power-up 

; initialize the RTC
   clr   temp
   rcall write_enable
   rcall read_trickle                  ; if already enabled then do not init again
   cpi   temp,$A5
   breq  no_clock_init
   rcall initialize_date_chip          ; Initialize the date chip to use the trickle charger.
no_clock_init:
   rcall get_time

; find out from RTC if we are displaying in 12 or 24 hour mode
   sbr   flags, (1 << MILITARY_FLAG)   ; set for 24hr mode
   lds   temp, hours_tens
   sbrc  temp, HR_12_BIT               ; check for 12/24hr mode, if clear then 12hr mode              
   cbr   flags, (1 << MILITARY_FLAG)   ; clear for 12hr mode

;   rcall write_auto_off                ; set auto time at startup so that power turns on for 1hr after power cycle 
   rcall delay10ms                     ; allow RC time constant for external trigger on power-up
   rcall get_settings                  ; retrieve saved clock settings (power, relay state and brightness)
   rcall update_alarm_ram              ; get alarm time from RTC RAM and store to local RAM
   rcall print_time                    ; update time with current brightness settings
   colons_on                           ; turn both colons on

   sei

;****************************************************************************************
;* Main program loop - cycle the puck here, main loop runs at about 3KHz

main:
   sbi   portb, LED
   rcall do_elements             ; PWM elements
   rcall do_power                ; turn boost regulator on or off
   rcall do_external_trigger     ; motion sensor
   rcall parse_ir_input          ; parse IR input when it is received 
   rcall do_brightness           ; adjust element brightness to suit ambient light
;   rcall get_temp                ; get temperature from sensor
   rcall auto_off_alarm          ; check if it is time to turn clock off 
   rcall get_time                ; get time from RTC and store to RAM
   rcall element_test            ; test mode to verify element functionality
   rcall desputter               ; randomize digits to desputter seldom used elements
   rcall print_time              ; display the time from RAM
   rcall check_alarm             ; check if it is time for alarm to sound
;   rcall check_snooze             ; check for snooze alarm to sound
   rcall check_alarm_off         ; check if it is time for alarm to turn off
   cbi   portb, LED
   rjmp  main

;****************************************************************************************
;* display firmware version
;* major version is displayed to hours and minor to minutes
display_version:
   rcall digits_off              ; turn display off

   sts   colbot0, brightness     ; turn on decimal
   sts   col_ext, zero
   sts   col_pm, zero
   sts   col_ai, zero
                                 
; display firmware major (single digit)
   ldi   temp, FW_VER_H
   sts   hours, temp

; display firmware minor (double digit)
   ldi   temp, FW_VER_L
   rcall separate                ; separate firmware minor into 2 digits
   sts   minutes, temp
   mov   temp, temp_2
   sts   minutes_tens, temp

   sbr   flags_2, (1 << DISP_ALRM_FLAG)   ; disable display update from RTC
   ret

;****************************************************************************************
;* toggle power to boost regulator
do_power:                         
   sbrs  flags, POWER_FLAG
   power_off                       
   sbrc  flags, POWER_FLAG
   power_on
   ret

;****************************************************************************************
;* turn clock power on when an input is detected on the external trigger
;* detects rising or falling edges on external trigger input
;* will work with normally open or normally closed relay contacts to trigger
do_external_trigger:
   in    temp, pind
   lds   temp_2, trig_in          ; load last level from RAM
   andi  temp, (1 << TRIG)       ; mask for trigger input
   cp    temp, temp_2             ; compare current input to last level
   breq  no_edge_trig
   sts   trig_in, temp           ; store current level
   ex_trig_off                   ; turn colon off when external trigger sensed
   sbr   flags, (1 << TRIGGER_FLAG) + (1 << POWER_FLAG) 
   rcall write_auto_off          ; trigger clock to stay on for another hour
no_edge_trig:
   ret

;****************************************************************************************
;* temperature sensor
;* clock 13 bits from LM74 into temp_2 and temp
;* clock out MSB first, only clock 13 bits, last 3 bits are not required
;* D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0 1 X X
;*
;* temperature is displayed as tens of degrees as hours
;* the lower colon designates the decimal
;* fractions of degrees are displayed as minutes
;* the upper colon designates degree symbol
get_temp:
   cbi   portc, TMP_CS
   clr   temp
   clr   temp_2
   ldi   cnt1, 13
temp_loop:
   sbi   portc, RTC_CLOCK
   sbis  pinc, RTC_IO
   clc
   sbic  pinc, RTC_IO
   sec   
   rol   temp
   rol   temp_2
   cbi   portc, RTC_CLOCK

   dec   cnt1
   brne  temp_loop

   sbi   portc, TMP_CS

; now temp_2 and temp contain 13 bit temperature value
; temperature value ranges from $00 07 to $4B 00
; corresponding to 0 to 150 degrees celcius, 128 per degree

   andi  temp_2, 0b00000111
   sts   seconds_tens, temp_2
   andi  temp, 0b00000111  
   sts   seconds, temp 
      
   ret

;****************************************************************************************
;* de-sputter tubes by randomly displaying digits
desputter:
   sbic  pind, button2              ; test clock if button pushed
   rjmp  skip_desputter
;   cpi   mode, DESPUTTER_MODE
;   brne  skip_desputter 

; display random digit to all elements (could be more effective to only do tens of digits)
   rcall random
   sts   seconds, temp
   rcall random
   sts   seconds_tens, temp
   rcall random
   sts   minutes, temp
   rcall random
   sts   minutes_tens, temp
   rcall random
   sts   hours, temp  
   rcall random
   sts   hours_tens, temp

skip_desputter:
   ret

;****************************************************************************************
;* test hardware by incrementing all digits to quickly scan through all elements
element_test:                         
   sbic  pind, button1              ; test clock if button pushed
   rjmp  skip_element_test

; replace all display RAM with seconds to quickly scan through all elements
   lds   temp, seconds
   sts   seconds_tens, temp
   sts   minutes, temp
   sts   minutes_tens, temp
   sts   hours, temp
   sts   hours_tens, temp

skip_element_test:
   ret

;****************************************************************************************
;* check if auto off time has occured, does not check tens of hours
;* this function turns the power to the display off when the alarm time is reached
auto_off_alarm:
   sbrs  flags, AUTO_OFF_FLAG    ; if auto off flag is set then auto power down
   rjmp  end_auto_off            ; else ignore

   cpi   mode, DESPUTTER_MODE    ; don't turn off during desputter (could randomly hit auto off time)
   breq  end_auto_off

   sbrc  flags_2, SET_FLAG       ; don't turn off display while setting time/date/timer/alarm
   rjmp  end_auto_off

   lds   temp, hours
   lds   temp_2, auto_hrs
   cp    temp, temp_2
   brne  end_auto_off

   lds   temp, minutes
   lds   temp_2, auto_min
   cp    temp, temp_2
   brne  end_auto_off

   lds   temp, minutes_tens
   lds   temp_2, auto_min_ten
   cp    temp, temp_2
   brne  end_auto_off

   lds   temp, seconds
   lds   temp_2, auto_sec
   cp    temp, temp_2
   brne  end_auto_off

   lds   temp, seconds_tens
   lds   temp_2, auto_sec_ten
   cp    temp, temp_2
   brne  end_auto_off

   cbr   flags, (1 << POWER_FLAG) + (1 << AUTO_OFF_FLAG)   ; turn power off after time-out, clear flag
   rcall save_settings              ; save power down state to RAM
end_auto_off:
   ret

;****************************************************************************************
;* turn buzzer off and toggle relay when alarm off minutes and seconds match current time
check_alarm_off:
   sbrs  flags_2, ALARM_EN_FLAG    ; only sound alarm if it has been enabled
   rjmp  end_check_alarm_off

   lds   temp, minutes
   lds   temp_2, alarm_off_min
   cp    temp, temp_2
   brne  end_check_alarm_off

   lds   temp, seconds
   lds   temp_2, alarm_off_sec
   cp    temp, temp_2
   brne  end_check_alarm_off

   lds   temp, seconds_tens
   lds   temp_2, alarm_off_sec_ten
   cp    temp, temp_2
   brne  end_check_alarm_off

   sbrc  flags_2, ALARM_OFF_FLAG    ; only toggle relay once
   rjmp  end_check_alarm_off

   buzzer_off                       ; turn buzzer off
   relay_off		                ; relay off
   sbr   flags_2, (1 << ALARM_OFF_FLAG)
   cbr   flags_2, (1 << ALARM_FLAG) ; turn off so that alarm can trigger again next time
end_check_alarm_off:
   ret

;****************************************************************************************
;* compare the display time with alarm time kept in RAM
;* only if alarm is enabled, alarm time is not currently being displayed or set
check_alarm:
   sbrs  flags_2, ALARM_EN_FLAG    ; only sound alarm if it has been enabled
   rjmp  end_check_alarm   

   sbrc  flags_2, DISP_ALRM_FLAG    ; if showing alarm time then do not allow alarm to sound
   rjmp  end_check_alarm

   sbrc  flags_2, SET_FLAG          ; don't triggler alarm while time/timer/alarm
   rjmp  end_check_alarm

   rcall get_time                   ; get time from RTC and store to RAM
                                    ; this is to avoid the possibility that the show_alarm expires
                                    ; right before check_alarm is called in the main loop

   lds   temp, hours_tens           ; compare display time with alarm time
   lds   temp_2, alarm_hrs_ten
   cp    temp, temp_2
   brne  end_check_alarm

   lds   temp, hours
   lds   temp_2, alarm_hrs
   cp    temp, temp_2
   brne  end_check_alarm

   lds   temp, minutes
   lds   temp_2, alarm_min
   cp    temp, temp_2
   brne  end_check_alarm

   lds   temp, minutes_tens
   lds   temp_2, alarm_mins_ten
   cp    temp, temp_2
   brne  end_check_alarm

   lds   temp, seconds
   lds   temp_2, alarm_sec
   cp    temp, temp_2
   brne  end_check_alarm

   lds   temp, seconds_tens
   lds   temp_2, alarm_sec_ten
   cp    temp, temp_2
   brne  end_check_alarm

   sbrc  flags_2, ALARM_FLAG        ; only handle relay once when alarm triggers
   rjmp  end_check_alarm

   buzzer_on                        ; turn buzzer on (will be turned off with "beep" when IR received)
   relay_on			                ; relay on
   sbr   flags_2, (1 << ALARM_FLAG)

; set time to turn alarm off 1 second later
   rcall add_1sec                   ; add one second to current time, result in temp
   sts   alarm_off_sec, temp
   lds   temp, seconds_tens
   sts   alarm_off_sec_ten, temp
   lds   temp, minutes
   sts   alarm_off_min, temp


   cbr   flags_2, (1 << ALARM_OFF_FLAG)

end_check_alarm:
   ret

;****************************************************************************************
;* compare the display time with snooze time kept in RAM
;* only if snooze flag is set, alarm time is not currently being displayed or set
check_snooze:
   sbrs  flags, SNOOZE_FLAG          ; don't triggler snooze alarm unless snooze flag set
   rjmp  end_check_snooze

;   rcall get_time                   ; get time from RTC and store to RAM
                                    ; this is to avoid the possibility that the show_alarm expires
                                    ; right before check_alarm is called in the main loop
   ; NOTE: get_time here takes too long for main loop (lowers PWM to 40% and causes noticeable flicker)
   ;       not calling it may cause snooze alarm to erroneously go off

   lds   temp, minutes              ; compare display time with alarm time
   lds   temp_2, snooze_min
   cp    temp, temp_2
   brne  end_check_snooze

   lds   temp, minutes_tens
   lds   temp_2, snooze_mins_ten
   cp    temp, temp_2
   brne  end_check_snooze

   lds   temp, seconds
   lds   temp_2, snooze_sec
   cp    temp, temp_2
   brne  end_check_snooze

   lds   temp, seconds_tens
   lds   temp_2, snooze_sec_ten
   cp    temp, temp_2
   brne  end_check_snooze

   buzzer_on                        ; turn buzzer on (will be turned off with "beep" when IR received)
   cbr   flags, (1 << SNOOZE_FLAG)  ; snooze is over, time to wake up, again

; set time to turn alarm off 1 minute ahead
   rcall add_1min                   ; add one minute to current time, result in temp
   sts   alarm_off_min, temp
   lds   temp, seconds
   sts   alarm_off_sec, temp
   lds   temp, seconds_tens
   sts   alarm_off_sec_ten, temp

   cbr   flags_2, (1 << ALARM_OFF_FLAG)

end_check_snooze:
   ret


;****************************************************************************************
;* read input IR command and parse
parse_ir_input:
   sbrs  flags, IR_FLAG
   rjmp  end_parse_ir 
   beep                             ; confirmation tone (valid IR command)
   ;cbr   flags_2, (1 << ALARM_FLAG) ; clear the alarm flag when IR recieved                        

   rcall ir_other_functions         ; one-button IR input functions
   rcall snooze                     ; snooze button pressed
   rcall set_brightness             ; volume up/down changes brightness
   rcall ir_toggle_power            ; toggle power on and off
   rcall ir_toggle_relay            ; toggle relay on and off
   rcall ir_set_clock               ; receive number buttons as time/alarm/timer setting          
   rcall ir_set_mode                ; press enter + mode number
   rcall save_settings              ; if settings (brightness, relay or power) have changed save them to RAM

end_parse_ir:
   cbr   flags, (1 << IR_FLAG)      ; command succesfully parsed
   ret

;****************************************************************************************
;* one-button IR input functions
ir_other_functions:
   cpi   ir_command, ENTER          ; if it is the ENTER button then exit
   breq  end_other_functions
   sbrc  flags, MENU_FLAG           ; if menu flag set then exit
   rjmp  end_other_functions
   sbrc  flags_2, SET_FLAG          ; if set time mode then exit
   rjmp  end_other_functions

   cpi   ir_command, 6              ; if "6" button was pushed then
   brne  PC+3
   rcall show_alarm                 ; show alarm time for 2.5 seconds
   rjmp  end_other_functions

   cpi   ir_command, 8              ; if "8" button was pushed then
   brne  PC+3
   rcall display_version            ; display firmware version
   rjmp  end_other_functions

   cpi   ir_command, 4              ; if "4" button was pushed then toggle display mode
   brne  skip_this
   rcall toggle_12_24hr             ; toggle time

   lds   temp, alarm_hrs_ten        ; move alarm time to actual time for manipulation
   sts   hours_tens, temp           ; by toggle_12_24hr function call
   lds   temp, alarm_hrs
   sts   hours, temp
   sbr   flags_2, (1 << SET_ALARM_FLAG)  ; now toggle alarm setting flag to indicate changing of alarm time
   rcall toggle_12_24hr
   cbr   flags_2, (1 << SET_ALARM_FLAG)  ; clear flag
   rjmp  end_other_functions
skip_this:
 
end_other_functions:
   ret

;****************************************************************************************
;* volume up/down changes brightness
;* check ambient brightness and adjust corresponding RAM variable
;* bright_ram is the brightness during bright ambient light, correspondingly dim_ram is 
;* for dim ambient light, this allows the display to be any brightness during either bright 
;* or dim ambient light
;* each volume up/down button press results in a divide or multiply by 2 of the PWM value
;* therefore the brightness range is 32 -> 100%
;*                                   16 -> 50%
;*                                    8 -> 25%
;*                                    4 -> 12.5%
;*                                    2 -> 6.25%
;*                                    1 -> 3.125%
set_brightness:
   cpi   ir_command, VOLUME_UP       ; is the command the volume up button?
   breq  do_bright_up
   cpi   ir_command, VOLUME_DOWN     ; is the command the volume down button?
   breq  do_bright_down
   ret                               ; else exit


do_bright_up:
   sbis  ACSR, ACO                   ; if ACO bit is set then it is dark
   rjmp  up_bright_ram
up_dim_ram:          ; ambient light is dim and want to increase display brightness
   lds   temp, dim_ram
   cpi   temp, 32
   brsh  dim_highest
   lsl   temp
   sts   dim_ram, temp
   ret
dim_highest:
   ldi   temp, 32
   sts   dim_ram, temp   
   ret

up_bright_ram:       ; ambient light is bright and want to increase display brightness                 
   lds   temp, bright_ram
   cpi   temp, 32
   brsh  bright_highest
   lsl   temp
   sts   bright_ram, temp
   ret
bright_highest:
   ldi   temp, 32
   sts   bright_ram, temp
   ret
   
do_bright_down:
   sbis  ACSR, ACO                   ; if ACO bit is set then it is dark
   rjmp  down_bright_ram
down_dim_ram:        ; ambient light is dim and want to decrease display brightness
   lds   temp, dim_ram
   cpi   temp, 2
   brlo  dim_lowest
   lsr   temp        ; logical shift will dim faster
   sts   dim_ram, temp
   ret
dim_lowest:
   ldi   temp, 1
   sts   dim_ram, temp
   ret

down_bright_ram:     ; ambient light is bright and want to decrease display brightness
   lds   temp, bright_ram
   cpi   temp, 2
   brlo  bright_lowest
   lsr   temp
   sts   bright_ram, temp
   ret
bright_lowest:
   ldi   temp, 1
   sts   bright_ram, temp
   ret

;****************************************************************************************
;* if snooze button is pressed then add 15min to current time and store in snooze time ram
snooze:
   cpi   ir_command, MUTE             ; snooze button
   brne  end_snooze

   lds   temp, seconds                ; get current time
   sts   snooze_sec, temp             ; store to snooze time in RAM

   lds   temp, seconds_tens
   sts   snooze_sec_ten, temp

   rcall add_15min
   sts   snooze_min, temp
   sts   snooze_mins_ten, temp_2

   sbr   flags, (1 << SNOOZE_FLAG)   ; enable snooze alarm
end_snooze:
   ret

;****************************************************************************************
;* read alarm setting from RAM and write to display RAM
;* with DISP_ALRM_FLAG set RAM will not be updated from the RTC
;* the DISP_ALRM_FLAG is cleared in timer0_handler after 2.56 seconds
show_alarm:

   lds   temp, alarm_sec           ; get alarm time from RAM
   sts   seconds, temp             ; store to display time in RAM

   lds   temp, alarm_sec_ten
   sts   seconds_tens, temp

   lds   temp, alarm_min
   sts   minutes, temp

   lds   temp, alarm_mins_ten
   sts   minutes_tens, temp

   lds   temp, alarm_hrs
   sts   hours, temp

   lds   temp, alarm_hrs_ten
   sts   hours_tens, temp

   sbr   flags_2, (1 << DISP_ALRM_FLAG)   ; disable display update from RTC

   ret

;****************************************************************************************
;* save clock brightness and status settings to RTC RAM
;* status register bits are defined as follows:
;*
;* [  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  ]
;* [  1  |  1  |  1  |  1  |  0  |ALRM |RELAY| PWR ]
;*
save_settings:

   lds   temp, bright_ram           ; get current bright setting
   rcall write_bright               ; write to RTC RAM

   lds   temp, dim_ram              ; get current dim setting
   rcall write_dim                  ; write to RTC RAM

   ldi   temp, $F0
   sbrc  flags, POWER_FLAG          ; get current power setting
   ori   temp, 0b11110001
   sbic  portd, RELAY               ; get current relay setting  
   ori   temp, 0b11110010
   sbrc  flags_2, ALARM_EN_FLAG     ; get current alarm setting  
   ori   temp, 0b11110100
   rcall write_status               ; write to RTC RAM

   ret

;****************************************************************************************
;* retrieve clock status settings from RTC RAM
;* status register bits are defined as follows:
;*
;* [  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  ]
;* [  1  |  1  |  1  |  1  |  0  |ALRM |RELAY| PWR ]
;*
;* check the data for corruption, if power was lost then RAM data is cleared
;* if data is out of range then use default values.
get_settings:
   rcall read_status               ; read status register from RTC RAM
   cpi   temp, 240                 ; if value is out of range then (RAM corrupt due to power loss)
   brsh  PC+2                      ; else data is still valid
   ldi   temp, 1                   ; then update with default values (power on, relay off, alarm off)
 
; update power flag
   sbrs  temp, 0                   
   cbr   flags, (1 << POWER_FLAG)
   sbrc  temp, 0                   
   sbr   flags, (1 << POWER_FLAG)
; update relay state
   sbrs  temp, 1                   
   cbi   portd, RELAY
   sbrc  temp, 1                   
   sbi   portd, RELAY
; update alarm enabled flag
   sbrs  temp, 2                   
   cbr   flags_2, (1 << ALARM_EN_FLAG)
   sbrc  temp, 2                   
   sbr   flags_2, (1 << ALARM_EN_FLAG)

   rcall read_bright               ; read bright register from RTC RAM
   cpi   temp, 32                  ; if value is out of range then (RAM corrupt due to power loss)
   brlo  PC+2                      ; else data is still valid
   ldi   temp, 32                  ; restore with default 100%
   sts   bright_ram, temp           

   rcall read_dim                  ; read dim register from RTC RAM
   cpi   temp, 32                  ; if value is out of range then (RAM corrupt due to power loss)
   brlo  PC+2                      ; else data is still valid
   ldi   temp, 1                   ; restore with default 3.125%
   sts   dim_ram, temp              

   ret


;****************************************************************************************
;* update brigtness settings for elements turned on by scanning elements RAM and
;* replacing currently stored PWM value with new PWM value, temp contains new brightness
;* elements with differing values are either turned off or in mid fade (will fade soon)

update_RAM:
   mov   temp_2, brightness         ; backup current brightness
   mov   brightness, temp           ; load new value

   ldi   ZL, low(element_ram_start)
   ldi   ZH, high(element_ram_start)
display_loop:
   ld    temp, Z                    ; get element PWM setting
   cp    temp, temp_2
   brne  PC+2
   st    Z, brightness              ; replace current PWM setting
   adiw  ZH:ZL, 1
   
   cpi   ZH, high(element_ram_end)
   brne  display_loop
   cpi   ZL, low(element_ram_end)
   brne  display_loop
   ret

;****************************************************************************************
;* while in set clock mode the first 6 IR commands in the correct range will be
;* considered time inputs starting from tens of hours and ending at seconds
;* in this mode the main loop will update the display such that inputs will be
;* echoed to the display
ir_set_clock:      
   sbrs  flags_2, SET_FLAG
   rjmp  end_set_clock               ; exit if not in set mode

set_clock_0:
   mov   temp, digit_count
   cpi   temp, 7
   breq  set_hours_tens
   cpi   temp, 6
   breq  set_hours
   cpi   temp, 5
   breq  set_minutes_tens
   cpi   temp, 4
   breq  set_minutes
   cpi   temp, 3
   breq  set_seconds_tens
   cpi   temp, 2
   breq  set_seconds

; get am/pm, button "0" or "1" respectively
   cpi   ir_command, 1        ; 1 -> pm
   brne  PC+2
   sbr   flags, (1 << AM_PM_FLAG)
   cpi   ir_command, 0        ; 0 -> am
   brne  PC+2
   cbr   flags, (1 << AM_PM_FLAG)
   rjmp  done_digit

set_seconds:
   cpi   ir_command, 10
   brsh  set_wrong_key
   sts   seconds, ir_command

   sbrc  flags, MILITARY_FLAG    ; if in 24hr mode then skip last input
   dec   digit_count
   rjmp  done_digit

set_hours_tens:
   sbrc  flags, MILITARY_FLAG    ; if set then in 24hr
   rjmp  set_hrs_tens_0

; 12hr valid entry check 
   cpi   ir_command, 2           ; 12hr mode only goes to 1
   rjmp  set_hrs_tens_1

; 24hr valid entry check
set_hrs_tens_0: 
   cpi   ir_command, 3           ; 24hr mode only goes to 2

set_hrs_tens_1:
   brsh  set_wrong_key
   sts   hours_tens, ir_command
   rjmp  done_digit

set_hours:
   sbrc  flags, MILITARY_FLAG    ; if set then in 24hr
   rjmp  set_hrs_0               ; do 24hr

; 12hr valid entry check
   lds   temp_2, hours_tens
   tst   temp_2                  ; if tens of hours is 1 then hours only goes to 2
   breq  set_hrs_1               ; else up to 9
   cpi   ir_command, 3           ; in 12hr mode hours can only go up to 2
   rjmp  set_hrs_2

; 24hr valid entry check
set_hrs_0:   
   lds   temp_2, hours_tens
   cpi   temp_2, 2
   brlo  set_hrs_1            ; if tens of hours is 1 then hours can be up to 9
   cpi   ir_command, 4        ; else hours can only go up to 3
   rjmp  set_hrs_2     
set_hrs_1:
   cpi   ir_command, 10
set_hrs_2:
   brsh  set_wrong_key
   sts   hours, ir_command
   rjmp  done_digit

set_seconds_tens:   
   cpi   ir_command, 6
   brsh  set_wrong_key
   sts   seconds_tens, ir_command
   rjmp  done_digit

set_minutes:   
   cpi   ir_command, 10
   brsh  set_wrong_key
   sts   minutes, ir_command
   rjmp  done_digit

set_minutes_tens:   
   cpi   ir_command, 6
   brsh  set_wrong_key
   sts   minutes_tens, ir_command

done_digit:
   dec   digit_count
   brne  end_set_clock              ; get all 6 digits before exiting set time mode

   sbrc  flags_2, SET_TIME_FLAG
   rcall write_time                 ; write time to RTC from RAM
   sbrc  flags_2, SET_ALARM_FLAG
   rcall write_alarm

set_wrong_key:
   cbr   flags_2, (1 << SET_FLAG) + (1 << SET_ALARM_FLAG) + (1 << SET_TIME_FLAG)
   colons_on                        ; make sure colons are on when exiting programming

end_set_clock:
   ret

;****************************************************************************************
;* write alarm time from time RAM to RTC RAM
;* then update alarm time RAM with values from RTC
write_alarm:
   lds   temp, hours
   lds   temp_2, hours_tens

   sbrc  flags, MILITARY_FLAG ; if set then do 24hr mode
   rjmp  wa_24hr_mode
; else do 12hr write
   sbr   temp_2, (1 << HR_12_BIT)      ; set MSB for 12hr mode
   sbrc  flags, AM_PM_FLAG             ; set am/pm flag
   sbr   temp_2, (1 << AM_PM_BIT)
   sbrs  flags, AM_PM_FLAG
   cbr   temp_2, (1 << AM_PM_BIT)

wa_24hr_mode:
   rcall merge_nybble               ; merge upper and lower nybbles for RTC time register
   rcall write_alarm_hours

   lds   temp, minutes
   lds   temp_2, minutes_tens
   rcall merge_nybble
   rcall write_alarm_min

   lds   temp, seconds
   lds   temp_2, seconds_tens
   rcall merge_nybble
   rcall write_alarm_sec

   rcall update_alarm_ram           ; get alarm time from RTC RAM and store to local RAM

   cbr   flags_2, (1 << SET_ALARM_FLAG)
   ret
;****************************************************************************************
;* get alarm time from RTC RAM and store to local RAM
;* the local copy of alarm time makes it simpler to manipulate
update_alarm_ram:
   rcall read_alarm_sec
   mov   temp_2, temp              ; split into upper and lower nybble 
   andi  temp, $0F
   swap  temp_2
   andi  temp_2, $0F               ; and separate
   sts   alarm_sec, temp
   sts   alarm_sec_ten, temp_2

   rcall read_alarm_min
   mov   temp_2, temp              
   andi  temp, $0F
   swap  temp_2
   andi  temp_2, $0F
   sts   alarm_min, temp
   sts   alarm_mins_ten, temp_2

   rcall read_alarm_hours
   mov   temp_2, temp              
   andi  temp, $0F
   swap  temp_2
   andi  temp_2, $0F
   sts   alarm_hrs, temp
   sts   alarm_hrs_ten, temp_2

   ret

;****************************************************************************************
;* write time from RAM to RTC
write_time:
   lds   temp, hours
   lds   temp_2, hours_tens

   sbrc  flags, MILITARY_FLAG ; if set then do 24hr mode
   rjmp  wt_24hr_mode
; else do 12hr write
   sbr   temp_2, (1 << HR_12_BIT)      ; set MSB for 12hr mode
   sbrc  flags, AM_PM_FLAG             ; set am/pm flag
   sbr   temp_2, (1 << AM_PM_BIT)
   sbrs  flags, AM_PM_FLAG
   cbr   temp_2, (1 << AM_PM_BIT)

wt_24hr_mode:
   rcall merge_nybble         ; merge upper and lower nybbles for RTC time register
   rcall write_hours

   lds   temp, minutes
   lds   temp_2, minutes_tens
   rcall merge_nybble
   rcall write_minutes

   lds   temp, seconds
   lds   temp_2, seconds_tens
   rcall merge_nybble
   rcall write_seconds
   ret

;****************************************************************************************
;* get current time from RAM, add 1hr to hours and store as off time in RAM
;* ignores tens of hours
write_auto_off:
   sbr   flags, (1 << AUTO_OFF_FLAG); set auto off flag
  
   rcall add_1hr                    ; adds one hour to current time, result in temp
   sts   auto_hrs, temp
   lds   temp, minutes
   sts   auto_min, temp
   lds   temp, minutes_tens
   sts   auto_min_ten, temp
   lds   temp, seconds
   sts   auto_sec, temp
   lds   temp, seconds_tens
   sts   auto_sec_ten, temp
   ret

;****************************************************************************************
;* combine units and tens of units for ease of manipulation, result in temp
howmany:
   clr   temp_2
   tst   temp
   breq  no_tens           ; no tens
howmany_loop:
   addi  temp_2, 10
   dec   temp
   brne  howmany_loop      ; overflow
no_tens:
   ret

;****************************************************************************************
;* add one hour to current time, the result:
;* temp will contain hours
;* temp_2 will contain tens of hours
 
add_1hr:
   lds   temp, hours_tens
   sbrs  flags, MILITARY_FLAG ; determine if 12hr mode
   andi  temp, $01            ; mask out 12hr mode bit and am/pm indicator
   rcall howmany              ; convert tens of hours to hours, result in temp_2
   lds   temp, hours
   add   temp, temp_2         ; now temp contains number of hours

   sbrs  flags, MILITARY_FLAG ; first determine 12/24hr mode
   rjmp  add_1_12hr
               
; do 24hr
   cpi   temp, 23
   breq  overflow_to_0
   inc   temp              ; else just add one hour
   rjmp  end_add_1hr
overflow_to_0:             ; 23hrs rolls over to 0hrs
   clr   temp
   rjmp  end_add_1hr

; do 12hr
add_1_12hr:
   cpi   temp, 12   
   breq  overflow_to_1
   inc   temp              ; else just add one hour
   rjmp  end_add_1hr

overflow_to_1:                       ; 12:00 rolls over to 1:00
   ldi   temp, 1

end_add_1hr:
   rcall separate          ; separate tens of hours and hours in temp register

   ret

;****************************************************************************************
;* add one min to current time, the result:
;* temp will contain minutes
;* temp_2 will contain tens of minutes
add_1min:
   lds   temp, minutes_tens
   rcall howmany           ; convert tens of minutes to minutes, result in temp_2
   lds   temp, minutes
   add   temp, temp_2      ; now temp contains number of minutes

   cpi   temp, 59
   breq  overflow_to_00
   inc   temp              ; else just add one minute
   rjmp  end_add_1min
overflow_to_00:            ; 59 minutes rolls over to 0 minutes
   clr   temp
end_add_1min:
   rcall separate          ; separate tens of minutes and minutes in temp register
   ret                     ; number of tens result in temp_2, remainder in temp

;****************************************************************************************
;* add one sec to current time, the result:
;* temp will contain seconds
;* temp_2 will contain tens of seconds
add_1sec:
   lds   temp, seconds_tens
   rcall howmany           ; convert tens of seconds to seconds, result in temp_2
   lds   temp, seconds
   add   temp, temp_2      ; now temp contains number of seconds

   cpi   temp, 59
   breq  overflow_to_00A
   inc   temp              ; else just add one seconds
   rjmp  end_add_1sec
overflow_to_00A:           ; 59 minutes rolls over to 0 minutes
   clr   temp
end_add_1sec:
   rcall separate          ; separate tens of seconds and seconds in temp register
   ret                     ; number of tens result in temp_2, remainder in temp


;****************************************************************************************
;* add 15 min to current time, the result:
;* temp will contain minutes
;* temp_2 will contain tens of minutes
add_15min:
   lds   temp, minutes_tens
   rcall howmany           ; convert tens of minutes to minutes, result in temp_2
   lds   temp, minutes
   add   temp, temp_2      ; now temp contains number of minutes

   cpi   temp, 45
   breq  overflow_to_00B   ; if 45 then overflow to 00
   cpi   temp, 45
   brlo  no_overflow  
   subi  temp, 60          ; if greater than 45 subtract 45
no_overflow:
   subi  temp, -15         ; else add 15 minute             
   rjmp  end_add_15min
overflow_to_00B:           ; 45 minutes rolls over to 0 minutes
   clr   temp
end_add_15min:
   rcall separate          ; separate tens of minutes and minutes in temp register
   ret                     ; number of tens result in temp_2, remainder in temp
;****************************************************************************************
;* determine how many tens of units are in temp
;* number of tens result in temp_2, remainder in temp
separate:
   clr   temp_2
   tst   temp
   breq  end_separate      ; no tens
separate_loop:
   inc   temp_2
   subi  temp, 10
   brcc  separate_loop     ; overflow
   addi  temp, 10          ; undo the overflow
   dec   temp_2
end_separate:
   ret


;****************************************************************************************
;* merge upper and lower nybbles for RTC time register
;* temp should contain hours, temp_2 should contain tens of hours
merge_nybble:
   andi  temp, $0F
   swap  temp_2
   andi  temp_2, $F0
   or    temp, temp_2
   ret

;****************************************************************************************
;* set mode
;* if the menu button is pressed then set the menu flag and start a timer
;* if a number button is pressed before the menu time-out then execute corresponding mode

ir_set_mode:
   cpi   ir_command, ENTER          ; if it is the ENTER button then set menu flag and...
   brne  not_menu_key
   sbr   flags, (1 << MENU_FLAG)
   colons_on                        ; make sure colons are on when exiting programming
   rjmp  end_set_mode

not_menu_key:
; if still in menu mode (menu mode times out after 2.55 sec) then get mode
   sbrs  flags, MENU_FLAG         
   rjmp  end_set_mode

   cpi   ir_command, 3              ; if "3" button was pushed then toggle alarm enable on/off
   brne  PC+3
   rcall toggle_alarm
   rjmp  end_set_mode

   cpi   ir_command, 0              ; if "0" then enter program mode
   brne  PC+2
   rjmp  THIRDBOOTSTART

; next two options for commands are set time/alarm so 
; turn off all digits, each digit will turn on as it is programmed
   rcall digits_off

   ldi   temp, 7                    ; if in 12hr mode then include the am/pm indicator
   mov   digit_count, temp          ; receive all 7 IR inputs
       
   cpi   ir_command, 1              ; if "1" button was pushed then do set time
   brne  PC+2
   sbr   flags_2, (1 << SET_TIME_FLAG)+(1 << SET_FLAG)    ; set time

   cpi   ir_command, 2              ; if "2" button was pushed then do set alarm
   brne  PC+2
   sbr   flags_2, (1 << SET_ALARM_FLAG)+(1 << ALARM_EN_FLAG)+(1 << SET_FLAG)    ; set and enable alarm

   cbr   flags, (1 << MENU_FLAG)    ; done with menu
end_set_mode:                       
   ret      

;****************************************************************************************
;* toggle alarm flag on/off
toggle_alarm:
   mov   temp, flags_2              ; make backup of flags before toggle
                                 
   sbrc  temp, ALARM_EN_FLAG        ; toggle flag
   cbr   flags_2, (1 << ALARM_EN_FLAG)
   sbrs  temp, ALARM_EN_FLAG         
   sbr   flags_2, (1 << ALARM_EN_FLAG)

   sts   col_ai, brightness         ; turn alarm indicator on so that it is not left off   
                                    ; it will be turned off later if alarm is not enabled
   ret

;****************************************************************************************
;* toggle 12/24hr flag, if set then display is in 24hr mode
toggle_12_24hr:
   sbrc  flags_2, DISP_ALRM_FLAG    ; if the alarm time is being displayed temporarily then
   rjmp  end_toggle_12_24hr         ; don't change 12/24hr and accidentally store alarm time as time

   sbrc  flags_2, SET_ALARM_FLAG    ; if updating alarm RAM then don't toggle display flag
   rjmp  no_toggle                  ; because we don't want to toggle the flag twice

   mov   temp_2, flags              ; save flags status in temp before toggle                                 
   sbrc  temp_2, MILITARY_FLAG      ; toggle flag
   cbr   flags, (1 << MILITARY_FLAG) 
   sbrs  temp_2, MILITARY_FLAG         
   sbr   flags, (1 << MILITARY_FLAG)
no_toggle:

   lds   temp, hours_tens
   sbrc  flags, MILITARY_FLAG       ; if set now then it was 12hr mode before toggle
   andi  temp, $01                  ; mask out 12hr mode bit and am/pm indicator
   rcall howmany                    ; convert tens of hours to hours, result in temp_2
   lds   temp, hours
   add   temp, temp_2               ; now temp contains number of hours

   sbrs  flags, MILITARY_FLAG
   rjmp  convert_to_12hr

convert_to_24hr:
   lds   temp_2, hours_tens         ; get am/pm bit
   sbrs  temp_2, AM_PM_BIT
   rjmp  was_am
was_pm:
   cpi   temp, 12
   breq  end_convert                ; 12:00pm -> 12:00hrs
   addi  temp, 12                   ; else add 12 hours to get 24hr mode
   rjmp  end_convert
was_am:                       
   cpi   temp, 12             
   brne  end_convert                ; if am then do nothing
   clr   temp                       ; else 12:00am -> 00:00 hrs
   rjmp  end_convert

convert_to_12hr:
   cpi   temp, 12                   ; 12:00hrs -> 12:00pm
   breq  do_12
   cpi   temp, 0                    ; 00:00hrs -> 12:00am
   breq  do_0
   cpi   temp, 12             
   brsh  do_sub_12                  ; greater than 12:00hrs -> subract 12hrs PM                               
   cbr   flags, (1 << AM_PM_FLAG)   ; else no change and AM
   rjmp  end_convert
do_sub_12:
   subi  temp, 12
   sbr   flags, (1 << AM_PM_FLAG)   ; set PM indicator             
   rjmp  end_convert
do_0:
   ldi   temp, 12
   cbr   flags, (1 << AM_PM_FLAG)   ; clear PM indicator
   rjmp  end_convert
do_12:
   sbr   flags, (1 << AM_PM_FLAG)   ; set PM indicator
   rjmp  end_convert

end_convert:
   rcall separate                   ; separate tens of hours and hours in temp register

   sbrc  flags, MILITARY_FLAG
   rjmp  do_24hr_write
; else do 12hr write
   sbr   temp_2, (1 << HR_12_BIT)   ; set MSB for 12hr mode
   sbrc  flags, AM_PM_FLAG          ; set am/pm flag
   sbr   temp_2, (1 << AM_PM_BIT)
   sbrs  flags, AM_PM_FLAG
   cbr   temp_2, (1 << AM_PM_BIT)
do_24hr_write:

   rcall merge_nybble               ; merge upper and lower nybbles for RTC time register

   sbrs  flags_2, SET_ALARM_FLAG
   rcall write_hours                ; write time to RTC from RAM
   sbrc  flags_2, SET_ALARM_FLAG
   rcall write_alarm_hours
   sbrc  flags_2, SET_ALARM_FLAG
   rcall update_alarm_ram           ; get alarm time from RTC RAM and store to local RAM

   colons_on                        ; in case they were blinked off during change

end_toggle_12_24hr:
   ret

;****************************************************************************************
;* toggle power on and off
;* toggles power flag bit and alarm flag bit, alarm flag bit is toggled so that if power
;* is turned on by IR the external trigger alarm will not turn display off
ir_toggle_power:
   cpi   ir_command, power          ; is the command the power button?
   brne  end_toggle

toggle_power:
   mov   temp, flags                ; save flags status in temp 
                                 
   sbrc  temp, POWER_FLAG           ; toggle flag
   cbr   flags, (1 << POWER_FLAG)
   sbrs  temp, POWER_FLAG         
   sbr   flags, (1 << POWER_FLAG)  

   cbr   flags, (1 << AUTO_OFF_FLAG) ; if using the remote to turn power on then disable display time-out
end_toggle:
   ret

;****************************************************************************************
;* toggle relay on and off
;* read relay port and toggle
;* in ir_toggle_relay check for correct IR command
;* also clear alarm flag so that relay toggles only once when alarm triggers

ir_toggle_relay:
   sbrc  flags_2, SET_FLAG          ; don't toggle if setting time/alarm
   rjmp  end_relay

   cpi   ir_command, 5              ; is the command the relay button?
   brne  end_relay

toggle_relay:
;   in    temp, pind
;   sbrs  temp, RELAY
   relay_on
;   sbrc  temp, RELAY     
   rcall  delayone                            
   relay_off

end_relay:
   ret

;****************************************************************************************
;* auto dimming - set element brightness according to ambient light and user preference
;*
;* the light sensor consists of photocell sensor voltage divider
;* the photocell range is approx. 1K in light, 20K medium and 100K in dark
;* the non-inverting input is 1.23V reference
;* the inverting input is the voltage divider, photocell to 5V, resistor to ground
;* 6K8 was chosen to generate 1.23V at nominal indoor ambient light

do_brightness:
   sbis  ACSR, ACO            ; if ACO bit is set then it is dark, so make elements dim
   rjmp  do_bright
do_dim:
   lds   temp, dim_ram
   rjmp  done_brighness
do_bright:
   lds   temp, bright_ram
done_brighness:
   rcall update_RAM           ; update brightness level of display

   ret

;****************************************************************************************
;* turn all digits in display off by loading RAM with value 10
;* 10 is a pointer for fading digits off
digits_off:
   ldi   temp, 10
   ldi   ZH, high(seconds)
   ldi   ZL, low(seconds)
   st    Z+, temp
   st    Z+, temp
   st    Z+, temp
   st    Z+, temp
   st    Z+, temp
   st    Z, temp
   ret

;****************************************************************************************
;* 5 bit PWM resolution (0 to 32), called from main loop at aproximately 1.6KHz
;* counter will count to 32 at approximately 50Hz PWM dutycycle
;* every interrupt pwmcount is incremented, up to 32 where it is reset to 0
;* when pwmcount is zero, DATA out to the shift register is set
;* each of the 64 element registers is compared to pwmcount, if it is equal
;* then the DATA output is cleared, the result is a PWM output for all 64 elements
;* 0 -> element 100%
;* 16 -> element 50%
;* 32 -> element 0% 
do_elements:
;   sbi   portb, LED
   ldi   ZL, low(element_ram_start)    ; init RAM pointer for element dutycycles 
   ldi   ZH, high(element_ram_start)      

   ldi   temp_2, 64           ; shift out all 64 bits
shift_1:
   cbi   portc, DATA          ; this will only get clocked if pwmcounter is 0
   tst   pwmcounter           ; turn all elements off at 0 (will get turned off at respective duty)
   breq  shift_2

; shift all bits out by carry
   ld    temp, Z+             ; load elements duty into temp (post increment)                  
   cp    temp, pwmcounter     ; generate carry if duty equal to pwmcounter
   brcc  PC+2                
   cbi   portc, DATA
   brcs  PC+2
   sbi   portc, DATA

shift_2:
; clock data out
   sbi   portc, CLK
   cbi   portc, CLK

   dec   temp_2                
   brne  shift_1             ; all 64 bits

   sbi   portc, STROBE
   cbi   portc, STROBE

   inc   pwmcounter          ; inrement every time timer int occurs
   cpi   pwmcounter, 32
   brlo  PC+2                ; only count to 32
   clr   pwmcounter

;   cbi   portb, LED
   ret

;****************************************************************************************
;* receive RC5 encoded IR signal from Sony remote control
;* data is received into ir_byte_hi and ir_byte_lo as follows
;*   bit | 15 | 14 | 13 | 12 | 11 | 10 |  9 |  8 |  7 |  6 |  5 |  4 |  3 |  2 |  1 |  0 |
;*         \    \  | S1 | S2 |  T | A4 | A3 | A2 | A1 | A0 | C5 | C4 | C3 | C2 | C1 | C0 |

; S1, S2 are start bits
; T is a toggle bit
; A4...A0 are address bits
; C5...C0 are command bits

; Command:           
; 0...9    Numbers 0...9 (channel select)
; 12       Power 
; 16       Volume + 
; 17       Volume - 
; 13       Mute
; 32       Channel +
; 33       Channel -
; 34       Previous Channel
; 53       ENTER
; 46       Menu

ir_receive:
   push  temp_2
   push  temp
   in    temp, sreg                 ; save status register
   push  temp
   push  cnt1                       ; so as not to screw up when returning from handler
   push  cnt2 

; 120ms timer to prevent IR interrupt from causing flicker during digit fading
   tst   ir_timer                   ; has command receive timer expired?
   brne  ir_skip                    ; if not then restart timer again such continuous IR stream is ignored

   ldi   temp, 13
   mov   counter, temp              ; 14 bit signal
ir_loop:
   rcall delay1ms                   ; delay to middle of bit
   rcall delay100us
   rcall delay100us
   rcall delay100us                 ; should be 1.35ms

   sbis  pind, IR_INPUT
   clc
   sbic  pind, IR_INPUT
   sec
   rol   ir_byte_lo
   rol   ir_byte_hi                 ; shift bits through carry
   dec   counter
   breq  ir_end

ir_wait:
   clr   temp                       ; temp will be used for wait time counter
   sbrs  ir_byte_lo, 0              ; check what last bit was
   rjmp  wait_for_hi
   sbrc  ir_byte_lo, 0
   rjmp  wait_for_lo                ; the do appropriate wait

wait_for_lo:
   sbis  pind, IR_INPUT
   rjmp  ir_loop                    ; when it goes low go back to sampling
   rcall delay100us                 ; else keep waiting
   inc   temp
   cpi   temp, 9                    ; 0.9ms is timeout period
   breq  ir_timeout                 ; straigt to end of receive
   rjmp  wait_for_lo                ; keep waiting
                              
wait_for_hi:
   sbic  pind, IR_INPUT
   rjmp  ir_loop                    ; when it goes low go back to sampling
   rcall delay100us                 ; else keep waiting
   inc   temp
   cpi   temp, 9                    ; 0.9ms is timeout period
   breq  ir_timeout
   rjmp  wait_for_hi                ; keep waiting

ir_end:
   mov   ir_command, ir_byte_lo     ; make a copy of IR command before it is overwritten by new IR
   sbr   flags, (1 << IR_FLAG)      ; indicate to main loop that an IR command was received

ir_skip:
   ldi   temp, 12                   ; load timer register for 120ms
   mov   ir_timer, temp             ; this is used to ignore all messages that arrive within 120ms
                                    ; such that only one command for each button push is recieved
                                    ; at the beginning of each button push

ir_timeout:                         ; if we timed out then command will still be clear

   pop   cnt2                       ; restore backed up registers
   pop   cnt1
   pop   temp                       
   out   sreg, temp
   pop   temp
   pop   temp_2
   reti

;****************************************************************************************
;* timer0 interrupt set for 10ms for
;* element fading - elements are updated at 100Hz, 10ms fade flag allows 1 second digit cross fade
;* menu button timing - 8 bit counter allows for up to 2.56 seconds of button timer
;* IR receive blocking - IR is blocked for 120ms after each IR message is received
;* display colon blinking - during set time

timer0_handler:
   push  temp                       ; save used registers and status
   in    temp, sreg           
   push  temp

; element fade timer
   sbr   fade_flags, (1 << FADE_FLAG)

; IR input message timer
   tst   ir_timer                   ; if ir_timer is already zero then do not dec again
   breq  PC+2
   dec   ir_timer                   ; else dec

; external trigger indicator
   sbrs  flags, TRIGGER_FLAG  
   rjmp  end_ext_timer              ; only increment when flag is set
   inc   ext_timer
   brne  end_ext_timer
   ex_trig_on                       ; turn colon back on
   cbr   flags, (1 << TRIGGER_FLAG) ; clear flag after 2.55 seconds
   clr   ext_timer                 ; reset alarm timer for next time flag is set
end_ext_timer:

; menu button timer
   sbrs  flags, MENU_FLAG  
   rjmp  end_menu_timer             ; only increment when flag is set
   inc   menu_timer
   brne  end_menu_timer
   cbr   flags, (1 << MENU_FLAG)    ; clear flag after 2.56 seconds
   clr   menu_timer                 ; reset menu timer for next time flag is set
end_menu_timer:

; show alarm timer (uses menu_timer register as the two events are exclusive)
   sbrs  flags_2, DISP_ALRM_FLAG  
   rjmp  end_alarm_timer            ; only increment when flag is set
   inc   menu_timer
   brne  end_alarm_timer
   cbr   flags_2, (1 << DISP_ALRM_FLAG) ; clear flag after 2.56 seconds
   clr   menu_timer                 ; reset menu timer for next time flag is set
   colons_on                        ; turn colons on after display firmware (turns some colons off)
end_alarm_timer:

; buzzer timer
   tst   buzzer_timer
   breq  end_buzzer_timer           ; only do buzzer timer when a value is loaded
   dec   buzzer_timer
   brne  end_buzzer_timer
   buzzer_off                       ; turn buzzer off after desired time
end_buzzer_timer:

; if alarm is enabled then blink alarm indicator colon
   sbrs  flags_2, ALARM_EN_FLAG
   rjmp  end_blink_ai
   ldi   temp, 100                  ; 1sec blink cadence 
   inc   colon_timer
   cp    colon_timer, temp
   brlo  end_blink_ai
   clr   colon_timer                ; reset timer
   lds   temp, col_ai               ; toggle alarm indicator colon
   tst   temp
   breq  turn_ai_on                 ; turn alarm indicator on
   sts   col_ai, zero               ; else turn it off
   rjmp  end_blink_ai
turn_ai_on:
   sts   col_ai, brightness
end_blink_ai:

; if in set time/alarm mode then blink colons
   sbrs  flags_2, SET_FLAG
   rjmp  end_blink                  ; if not in set time mode then do not blink

blink_colon:
   ldi   temp, 50                   ; 0.5sec blink cadence for set time mode 
   inc   colon_timer
   cp    colon_timer, temp
   brlo  end_blink
   clr   colon_timer                ; reset timer
   lds   temp, colbot0              ; toggle colons (make sure colbot0 is not cleared elsewhere)
   tst   temp
   breq  turn_colon_on              ; turn all colons on
   colons_off
   rjmp  end_blink
turn_colon_on:                      ; turn all colons off
   colons_on
end_blink:

; reset timer 0 
end_timer0:
   ldi   temp, (255 - 78)           ; reload timer 0 counter
   out   TCNT0, temp

   pop   temp
   out   sreg, temp
   pop   temp
   reti

;****************************************************************************************                                          
;* RTC subroutines
;* Contains all the subroutines for placeing a date into the DS 1302 chip attached to 
;* PORTA and read the date out and place it in RAM at the addresses designated in
;* the data segement. The subroutines are grouped by transmission and reception.

RTC_wr_cmd:
   sbi   PORTC, RTC_RESET  ; set reset high for reading/writing clock
RTC_wr_byte:
   push  temp_2
   push  temp
   ldi   temp_2, 8        ; count for bits in register
   sbi   DDRC, RTC_IO     ; make RTC_IO an output
rtc_wr_loop:
   sbrc  temp,0           ; if bit 0 in temp is set,
   sbi   PORTC, RTC_IO    ; turn on RTC I/O
   sbrs  temp,0           ; if bit 0 in temp is clear,
   cbi   PORTC, RTC_IO    ; turn off RTC I/O
   sbi   PORTC, RTC_CLOCK ; clock up
   nop
   nop
   cbi   PORTC, RTC_CLOCK ; clock down
   lsr   temp             ; shift to the next bit
   dec   temp_2           ; count the one we just sent
   brne  rtc_wr_loop      ; and go back if not done
   pop   temp
   pop   temp_2
   ret

;--------------
; RTC_rd_byte.  Reads the byte off the IO line and stores the value in the temp register.  
RTC_rd_byte:
   push  temp_2

   sbi   PORTC, RTC_IO    ; to set no pullup
   cbi   DDRC, RTC_IO     ; make an input
   ldi   temp_2, 8        ; get ready to count bits coming in
getdataloop:
cli
   sec                    ; presuppose bit = 1
   sbis  PINC, RTC_IO     ; but if IO clear
   clc                      ; then reset bit to 0
   sbi   PORTC, RTC_CLOCK ; raise clock
   nop
   nop
   cbi   PORTC, RTC_CLOCK ; drop clock
   ror   temp             ; 1
   dec   temp_2           ; 1
sei
   brne  getdataloop      ; 1
   pop   temp_2           ; 2
   ret

;--------------
; Subroutines for sending values to each of the registers.
; To use them you load the value you wish to send to the register on 
; the DS1302 into the temp register then make the appropriate rcall.

write_trickle:
   ldi   temp_2, WR_TRICKLE
   rjmp  writeRTCreg
write_enable:
   ldi   temp_2, WR_CONTROL
   rjmp  writeRTCreg
write_day:
   ldi   temp_2, WR_DAY
   rjmp  writeRTCreg
write_year:
   ldi   temp_2, WR_YEAR
   rjmp  writeRTCreg
write_month:
   ldi   temp_2, WR_MONTH
   rjmp  writeRTCreg
write_date:
   ldi   temp_2, WR_DATE
   rjmp  writeRTCreg
write_hours:
   ldi   temp_2, WR_HOUR
   rjmp  writeRTCreg
write_minutes:
   ldi   temp_2, WR_MIN
   rjmp  writeRTCreg
write_seconds:
   ldi   temp_2, WR_SEC
   rjmp  writeRTCreg
write_alarm_sec:
   ldi   temp_2, WR_SEC_ALARM
   rjmp  writeRTCreg
write_alarm_min:
   ldi   temp_2, WR_MIN_ALARM
   rjmp  writeRTCreg
write_alarm_hours:
   ldi   temp_2, WR_HOUR_ALARM
   rjmp  writeRTCreg
write_status:
   ldi   temp_2, WR_STATUS
   rjmp  writeRTCreg
write_bright:
   ldi   temp_2, WR_BRIGHT
   rjmp  writeRTCreg
write_dim:
   ldi   temp_2, WR_DIM
   rjmp  writeRTCreg

writeRTCreg:
   push  temp           ; Save value so it isn't changed in the write command subroutine.

   mov   temp, temp_2   ; Move the command into the temp registry to send.
   rcall RTC_wr_cmd
   pop   temp           ; Value to be sent written into the DS1302 Register.
   rcall RTC_wr_byte
   cbi   PORTC, RTC_RESET    ; Lower the reset pin      
   ret
;------------
; Read Values are returned in the temp register.
read_trickle:
   ldi   temp, RD_TRICKLE
   rjmp  readRTCreg
read_enable:
   ldi   temp, RD_CONTROL
   rjmp  readRTCreg
read_day:
   ldi   temp, RD_DAY
   rjmp  readRTCreg
read_month:
   ldi   temp, RD_MONTH
   rjmp  readRTCreg
read_year:
   ldi   temp, RD_YEAR
   rjmp  readRTCreg
read_seconds:
   ldi   temp, RD_SEC
   rjmp  readRTCreg
read_minutes:
   ldi   temp, RD_MIN
   rjmp  readRTCreg
read_hours:
   ldi   temp,RD_HOUR
   rjmp  readRTCreg
read_date:
   ldi   temp, RD_DATE
   rjmp  readRTCreg
read_alarm_sec:
   ldi   temp, RD_SEC_ALARM
   rjmp  readRTCreg
read_alarm_min:
   ldi   temp, RD_MIN_ALARM
   rjmp  readRTCreg
read_alarm_hours:
   ldi   temp, RD_HOUR_ALARM
   rjmp  readRTCreg
read_status:
   ldi   temp, RD_STATUS
   rjmp  readRTCreg
read_bright:
   ldi   temp, RD_BRIGHT
   rjmp  readRTCreg
read_dim:
   ldi   temp, RD_DIM
   rjmp  readRTCreg

readRTCreg:
   rcall RTC_wr_cmd
   rcall RTC_rd_byte
   cbi   PORTC, RTC_RESET    ; Lower the reset pin
   ret
;------- Start initialize_date_chip --------------- 
; Enable the trickle charger by writing 1010 to the high word of the trickle charger register
; and write to the 0101 
initialize_date_chip: 
   ldi   temp, 0            ; Write a 0 to the Control register to disable write protection of the registers.
   rcall write_enable
   ldi   temp, 0            ; Write a 0 to the seconds register to enable the clock.
   rcall write_seconds
   ldi   temp, $A5          ; Set trickle charger to charge as quickly as possible.
   rcall write_trickle
   ret

;****************************************************************************************                                          
;* Subroutine for getting and storing the date from the DS 1302
;* Stores the date read into the RAM location indexed by the Z index register.
;* This register must be pointing to the desired location before the subroutine is entered.
;* Reading out the date from the DS1302 takes exactly 824 cycles or 103 us (0.103 ms) including rcall and ret time. 
get_time:
   ;  The date will be read using the calendar burst mode.  This mode sends the 8 calendar bytes
   ;  consecutively starting with bit 0 of address 0 or Seconds.
   ;
   ;   Read Clock burst mode command.
   ;   | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
   ;    -------------------------------
   ;   | 1 | 0 | 1 | 1 | 1 | 1 | 1 | 1 |
   ;    -------------------------------
   ; Read out the date from the DS 1302 in clock mode.

   sbrc  flags_2, SET_FLAG       ; if in set clock mode then skip RTC
   rjmp  end_get_time

   sbrc  flags_2, DISP_ALRM_FLAG ; if in display alarm time mode then skip RTC
   rjmp  end_get_time

   push  ZL
   push  ZH

   ldi   ZL, low(seconds)
   ldi   ZH, high(seconds)

   ldi   temp, RD_BURST   ; Load the burst read mode command.
   rcall RTC_wr_cmd     ; Burst read to the DS1302 chip

   ; Wait tcdd time should be covered by the time for switching the bit direction
   ;   and the ret from the final bit send. 
   ; This subroutine takes up ? uS including rcall.
receive_clock_burst:
   ldi   cnt2, 8                  ; Load a counter for looping to receive the first 7 bytes of the burst clock.  IE Clock values.
receive_byte_ds1302_loop:
   rcall RTC_rd_byte
   mov   temp_2, temp             ; split into upper and lower nybble 
   andi  temp, $0F                ; and separate
   swap  temp_2
   andi  temp_2, $0F              
   st    Z+, temp                 ; store the values to RAM sequencially
   st    Z+, temp_2               
   dec   cnt2
   brne  receive_byte_ds1302_loop ; Get next byte.
   cbi   PORTC, RTC_RESET    ; Lower the reset pin
   ; Now the seconds value is stored in register date1.
   pop   ZH
   pop   ZL

end_get_time:
   ret 

;****************************************************************************************                                          
;* get the time stored in RAM and display on corresponding elements
;* only update time every time the fade_flag is set
;* the fade_flag controls the rate of digit fading
print_time:

   sbrs  fade_flags, FADE_FLAG
   rjmp  skip_print_time

   rcall print_seconds
   rcall print_sec_ten

   rcall print_minutes
   rcall print_min_ten

   rcall print_hours
   rcall print_hr_ten

skip_print_time:
   cbr   fade_flags, (1 << FADE_FLAG)
   ret

;***********************************************************************************************
;* updates seconds if there is a change
print_seconds:
   sbrc  fade_flags, SECONDS_FLAG    ; if in mid-fade then continue till complete
   rjmp  do_seconds_fade

   lds   temp, seconds
   sts   cur_sec, temp

; check seconds and only update display when there is a change
   lds   temp_2, last_sec 
   cp    temp, temp_2
   breq  end_print_seconds

   mov   seconds_pwm, brightness          ; initially start at brightness setting
   sbr   fade_flags, (1 << SECONDS_FLAG)  ; indicate that we are busy fading 

do_seconds_fade:
; fade current digit up
   lds   temp, cur_sec
   rcall get_ram_pointer      ; point to element in RAM with Z
   mov   temp, seconds_pwm
   sub   temp, brightness
   neg   temp
   st    Z, temp              
; fade last digit down
   lds   temp, last_sec       ; what was last displayed
   rcall get_ram_pointer      ; point to element in RAM with Z
   mov   temp, seconds_pwm
   st    Z, temp
   dec   seconds_pwm
   brpl  end_print_seconds

   cbr   fade_flags, (1 << SECONDS_FLAG) ; done fading
   lds   temp, cur_sec   
   sts   last_sec, temp       ; current seconds becomes last seconds
end_print_seconds:
   ret

;***********************************************************************************************
;* updates tens of seconds if there is a change
print_sec_ten:
   sbrc  fade_flags, SEC_TEN_FLAG    ; if in mid-fade then continue till complete
   rjmp  do_sec_tens_fade

   lds   temp, seconds_tens
   sts   cur_sec_ten, temp

; check seconds and only update display when there is a change
   lds   temp_2, last_sec_ten 
   cp    temp, temp_2
   breq  end_print_sec_ten

   mov   sec_ten_pwm, brightness          ; initially start at brightness setting
   sbr   fade_flags, (1 << SEC_TEN_FLAG)  ; indicate that we are busy fading 

do_sec_tens_fade:
; fade current digit up
   lds   temp, cur_sec_ten    ; what was is displayed
   addi  temp, 12             ; offset to tens of seconds
   rcall get_ram_pointer      ; point to element in RAM with Z
   mov   temp, sec_ten_pwm
   sub   temp, brightness
   neg   temp
   st    Z, temp
; fade last digit down
   lds   temp, last_sec_ten   ; what was last displayed
   addi  temp, 12
   rcall get_ram_pointer      ; point to element in RAM with Z
   mov   temp, sec_ten_pwm
   st    Z, temp
   dec   sec_ten_pwm
   brpl  end_print_sec_ten

   cbr   fade_flags, (1 << SEC_TEN_FLAG) ; done fading
   lds   temp, cur_sec_ten   
   sts   last_sec_ten, temp   ; current tens of seconds becomes last tens of seconds
end_print_sec_ten:
   ret 

;***********************************************************************************************
;* updates minutes if there is a change
print_minutes:
   sbrc  fade_flags, MINUTES_FLAG    ; if in mid-fade then continue till complete
   rjmp  do_minutes_fade

   lds   temp, minutes
   sts   cur_min, temp

; check seconds and only update display when there is a change
   lds   temp_2, last_min 
   cp    temp, temp_2
   breq  end_print_minutes

   mov   minutes_pwm, brightness          ; initially start at brightness setting
   sbr   fade_flags, (1 << MINUTES_FLAG)  ; indicate that we are busy fading

do_minutes_fade:
; fade current digit up
   lds   temp, cur_min        ; what was is displayed
   addi  temp, 24             ; offset to minutes
   rcall get_ram_pointer      ; point to element in RAM with Z
   mov   temp, minutes_pwm
   sub   temp, brightness
   neg   temp
   st    Z, temp
; fade current digit down
   lds   temp, last_min       ; what was last displayed
   addi  temp, 24
   rcall get_ram_pointer      ; point to element in RAM with Z           
   mov   temp, minutes_pwm
   st    Z, temp
   dec   minutes_pwm
   brpl  end_print_minutes

   cbr   fade_flags, (1 << MINUTES_FLAG) ; done fading
   lds   temp, cur_min   
   sts   last_min, temp       ; current minutes becomes last minutes
end_print_minutes:
   ret

;***********************************************************************************************
;* updates tens of minutes if there is a change
print_min_ten:
   sbrc  fade_flags, MIN_TEN_FLAG    ; if in mid-fade then continue till complete
   rjmp  do_min_ten_fade

   lds   temp, minutes_tens
   sts   cur_min_ten, temp

; check seconds and only update display when there is a change
   lds   temp_2, last_min_ten 
   cp    temp, temp_2
   breq  end_print_min_ten

   mov   min_ten_pwm, brightness          ; initially start at brightness setting
   sbr   fade_flags, (1 << MIN_TEN_FLAG)  ; indicate that we are busy fading

do_min_ten_fade:
; fade current digit up
   lds   temp, cur_min_ten    ; what is displayed
   addi  temp, 36             ; offset to tens of minutes
   rcall get_ram_pointer      ; point to element in RAM with Z
   mov   temp, min_ten_pwm
   sub   temp, brightness
   neg   temp
   st    Z, temp
; fade current digit down
   lds   temp, last_min_ten   ; what was last displayed
   addi  temp, 36
   rcall get_ram_pointer      ; point to element in RAM with Z
   mov   temp, min_ten_pwm
   st    Z, temp
   dec   min_ten_pwm
   brpl  end_print_min_ten

   cbr   fade_flags, (1 << MIN_TEN_FLAG)  ; done fading
   lds   temp, cur_min_ten   
   sts   last_min_ten, temp   ; current tens of minutes becomes last tens of minutes
end_print_min_ten:
   ret 

;***********************************************************************************************
;* updates hours if there is a change
print_hours:
   sbrc  fade_flags, HOURS_FLAG    ; if in mid-fade then continue till complete
   rjmp  do_hours_fade

   lds   temp, hours
   sts   cur_hr, temp

; check seconds and only update display when there is a change
   lds   temp_2, last_hr
   cp    temp, temp_2
   breq  end_print_hours

   mov   hours_pwm, brightness          ; initially start at brightness setting
   sbr   fade_flags, (1 << HOURS_FLAG)  ; indicate that we are busy fading

do_hours_fade:
; fade current digit up
   lds   temp, cur_hr        ; what is displayed
   addi  temp, 48             ; offset to hours
   rcall get_ram_pointer      ; point to element in RAM with Z
   mov   temp, hours_pwm
   sub   temp, brightness
   neg   temp
   st    Z, temp
; fade current digit down
   lds   temp, last_hr        ; what was last displayed
   addi  temp, 48
   rcall get_ram_pointer      ; point to element in RAM with Z            
   mov   temp, hours_pwm
   st    Z, temp
   dec   hours_pwm
   brpl  end_print_hours

   cbr   fade_flags, (1 << HOURS_FLAG)  ; done fading
   lds   temp, cur_hr   
   sts   last_hr, temp       ; current hours becomes last hours
end_print_hours:
   ret

;***********************************************************************************************
;* updates tens of hours if there is a change
;* hours register from RTC:
;*          bit     7    6    5    4    3    2    1    0
;*                _______________________________________
;* 12 hour mode  |  1 |  0 | A/P | HR |       HR         |
;*               |_______________________________________|
;*                _______________________________________
;* 24 hour mode  |  0 |  0 | HR  | HR |       HR         |
;*               |_______________________________________|

print_hr_ten:
   sbrc  fade_flags, HRS_TEN_FLAG    ; if in mid-fade then continue till complete
   rjmp  do_hrs_ten_fade

   lds   temp, hours_tens

; in test mode display in 24hr mode such that all digits are displayed
   sbis  pind, button1
   rjmp  do_normal

; if in set time mode then don't blank leading zero
   sbrc  flags_2, SET_FLAG       
   breq  do_normal

; now check for 12/24hr mode
   sbrc  flags, MILITARY_FLAG
   rjmp  do_normal          ; else display value as is
do_12_hour:
   sbrs  temp, 1            ; if set then PM
   rjmp  do_am
   sbrc  temp, 1            ; if set then PM
   rjmp  do_pm
do_pm:                      ; turn PM indicator on
   sts   col_pm, brightness
   rjmp  done_am_pm        
do_am:                      ; turn PM indicator off 
   sts   col_pm, zero     
done_am_pm:
   andi  temp, $01          ; mask for tens of hours
; do leading zero blanking for 12hr mode
   tst   temp
   breq  PC+2
   rjmp  do_normal
   ldi   temp, 10             ; now over write temp with offset for blank
do_normal:
   sts   cur_hr_ten, temp

; check seconds and only update display when there is a change
   lds   temp_2, last_hr_ten 
   cp    temp, temp_2
   breq  end_print_hr_ten

   mov   hrs_ten_pwm, brightness          ; initially start at brightness setting
   sbr   fade_flags, (1 << HRS_TEN_FLAG)  ; indicate that we are busy fading

do_hrs_ten_fade:
; fade current digit up
   lds   temp, cur_hr_ten   ; what is displayed
   addi  temp, 60           ; offset to tens of hours
   rcall get_ram_pointer    ; point to element in RAM with Z
   mov   temp, hrs_ten_pwm
   sub   temp, brightness
   neg   temp
   st    Z, temp
; fade current digit down
   lds   temp, last_hr_ten  ; what was last displayed
   addi  temp, 60
   rcall get_ram_pointer    ; point to element in RAM with Z
   mov   temp, hrs_ten_pwm
   st    Z, temp
   dec   hrs_ten_pwm
   brpl  end_print_hr_ten

   cbr   fade_flags, (1 << HRS_TEN_FLAG)  ; done fading
   lds   temp, cur_hr_ten   
   sts   last_hr_ten, temp  ; current tens of hours becomes last tens of hours
end_print_hr_ten:
   ret 

;***********************************************************************************************
;* loads Z with correct address in RAM for corresponding element in temp register
;* enter here with the number of seconds in temp register (0 - 9)- no offset
;*                               tens of seconds   - offset of 12
;*                               minutes           - offset of 24
;*                               tens of minutes   - offset of 36
;*                               hours             - offset of 48
;*                               tens of hours     - offset of 60
;* the number of seconds is used to look up the offset in RAM
;* Z then points to the corresponding element
get_ram_pointer:
   ldi   ZL, LOW(TABLE*2)
   ldi   ZH, HIGH(TABLE*2)
   add   ZL, temp            ; offset to correct element to turn on
   adc   ZH, zero            ; (16-bit add)
   lpm                       ; loads first value from table according to Z register
   mov   temp, R0            ; now temp contains the correct offset in RAM for the element to turn on

   ldi   ZL, low(element_ram_start)    ; point to currently increasing element 
   ldi   ZH, high(element_ram_start)
   add   ZL, temp            ; offset in RAM to correct element register
   adc   ZH, zero            ; (16-bit add)        
   ret

TABLE:
;* element offset in RAM
;* note that program memory is implemented in 16-bit words and therefore the table
;* rows must have an even number of bytes to fill word boundaries

;* the rows of this table are the 6 tubes
;* the columns are the tube elements 0 to 9, 10 and 11 are blanks

.ifdef CLOCK2
;      0   1   2   3   4   5   6   7   8   9  10  11
.db   14,  7,  6,  4,  3,  2,  1,  0,  5, 15, 64, 64         ; seconds
.db   20, 13, 12,  9, 22, 23, 11, 10,  8, 21, 64, 64         ; tens of seconds
.db   27, 18, 19, 30, 25, 24, 28, 29, 31, 26, 64, 64         ; minutes
.db   46, 39, 38, 36, 35, 34, 33, 32, 37, 47, 64, 64         ; tens of minutes
.db   49, 40, 41, 43, 52, 53, 54, 55, 42, 48, 64, 64         ; hours
.db   59, 50, 51, 62, 57, 56, 60, 61, 63, 58, 64, 64         ; tens of hours
.endif

.ifdef CLOCK3
;      0   1   2   3   4   5   6   7   8   9  10  11
.db    0,  1,  2,  3, 15, 14,  7,  6,  5,  4, 64, 64         ; seconds
.db   10, 11, 23, 22, 21, 20, 13, 12,  8,  9, 64, 64         ; tens of seconds
.db   29, 28, 24, 25, 26, 27, 18, 19, 31, 30, 64, 64         ; minutes
.db   32, 33, 34, 35, 47, 46, 39, 38, 37, 36, 64, 64         ; tens of minutes
.db   55, 54, 53, 52, 48, 49, 40, 41, 42, 43, 64, 64         ; hours
.db   61, 60, 56, 57, 58, 59, 50, 51, 63, 62, 64, 64         ; tens of hours     

.endif

;****************************************************************************************
; generates a 16-bit random number in rand_hi and rand_lo
random:

   mov   temp, rand_hi      ; if current random is 0000, make it 00FFH
   or    temp, rand_lo
   brne  next
   com   rand_lo
next:
   sbrs  rand_hi, 6         ; hi.7 = hi.7 xor hi.6
   ldi   temp, $00
   sbrc  rand_hi, 6
   ldi   temp, $80
   eor   rand_hi, temp

   sbrs  rand_hi, 4         ; hi.7 = hi.7 xor hi.4
   ldi   temp, $00
   sbrc  rand_hi, 4
   ldi   temp, $80
   eor   rand_hi, temp

   sbrs  rand_lo, 3         ; hi.7 = hi.7 xor lo.3
   ldi   temp, $00
   sbrc  rand_lo, 3
   ldi   temp, $80
   eor   rand_hi, temp

   sbrc  rand_hi, 7        ; test MSB
   sec                     ; set carry flag
   sbrs  rand_hi, 7        ; test MSB
   clc                     ; else clear carry flag
           
   rol   rand_lo           ; double left shift
   rol   rand_hi

   mov   temp, rand_hi
   andi  temp, 0b00001111  ; mask to 0-15
   cpi   temp, 10          ; redo random till lower than 10
   brsh  random

   ret

;****************************************************************************************
;* write to EEPROM
;* eeprom address is loaded in ee_addr and data is stored in temp before calling read_ee
  
;write_ee:
;   cli                              ; no interruptions during EE write
   
;   sbic  EECR, EEWE                 ; poll to see if EEPROM is ready
;   rjmp  write_ee

;   clr   temp_2
;   out   EEARH, temp                ; not using high address
;   out   EEARL, ee_addr             ; write EEPROM address to EEARL
;   out   EEDR, temp                 ; write data to EEPROM

;   ldi   temp, (1 << EEMWE) + (0 << EEWE)    ; set the master write enable and simultaneoustly clear write enable
;   out   EECR, temp
;   sbi   EECR, EEMWE                ; set the master write enable, EEMWE bit
;   sbi   EECR, EEWE                 ; set the write enable, EEWE bit

;   clr   temp   
;   out   EEARH, temp                ; point EE address to zero when done
;   out   EEARL, temp                ; point EE address to zero when done

;   sei
;   ret

;****************************************************************************************
;* read EEPROM
;* eeprom address is loaded in ee_addr before calling read_ee
;* data is stored in temp upon exit

;read_ee:   
;   sbic  EECR, EEWE                 ; poll to see if EEPROM is ready
;   rjmp  read_ee

;   clr   temp
;   out   EEARH, temp
;   out   EEARL, ee_addr             ; write EEPROM address to EEAR

;   sbi   EECR, EERE                 ; set the read enable, EERE bit
;   in    temp, EEDR                 ; read the data

;   ret


;****************************************************************************************                                          
;* Delay routines for 8 MHz clock 

delayone:
   rcall delayhalf
delayhalf:
   rcall delay100ms                 ;checked ok on Feb 19, 2001.
   rcall delay200ms
delay200ms:
   rcall delay100ms
delay100ms:
   rcall delay50ms
delay50ms:
   rcall delay10ms
   rcall delay20ms
delay20ms:
   rcall delay10ms
delay10ms:
   ldi   cnt1,100
delay5ms:
   ldi   cnt1, 50
d10ms:
   rcall delay100us
   dec   cnt1
   brne  d10ms
   ret
delay1ms:
   ldi   cnt1,10
d1ms:
   rcall delay100us
   dec   cnt1
   brne  d1ms
   ret
delay100us:
   ldi   cnt2,198
c1loop:
   nop
   dec   cnt2
   brne  c1loop
   ret

;****************************************************************************************
;*End of Program
