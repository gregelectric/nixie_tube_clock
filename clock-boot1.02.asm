;* File:         Nixie Clock 2 Boot - boot loader for MEGA8515
;* Author:       Greg Powell
;*               
;* Date Started: 5 April 2005    
;*
;* Revision History:
;* 0.0          2004 - started with AVR Freaks Design Note #032
;* 1.0    6 Apr 2005 - boot code supports the following instruction set:
;* 1.01  19 May 2005 - Fixed "write page" with "rjmp carriage_return" at end
;* 1.02  29 Aug 2005 - moved boot pin check to after port init

;      Operation                 CMD      DATA         Response
;------------------------------- ---      -------      -----------
;Auto Increment Address           a                    dd
;Set Address                      A        ah al                CR
;Check Block Support              b        Y 2*dd
;Start Block Flash Load           B        2*dd F               CR
;                                          n*dd
;Start Block EEPROM Load          B        2*dd E               CR
;                                          n*dd
;Write Program Memory, Low Byte   c        dd                   CR
;Write Program Memory, High Byte  C        dd                   CR
;Read Data Memory                 d                    dd
;Write Data Memory                D        dd                   CR
;Chip Erase                       e                             CR
;Exit Bootloader                  E                             CR
;Read Fuse Bits                   F                    dd
;Start Block Flash Read           g        2*dd F      n*dd
;Start Block EEPROM Read          g        2*dd E      n*dd
;Write Lock Bits                  l        dd                   CR
;Leave Programming Mode           L                             CR
;Issue Page Write                 m                             CR
;Read High Fuse Bits              N                    dd
;Enter Programming Mode           P                             CR
;Return Programmer Type           p                    dd
;Read Extended Fuse Bits          Q                    dd
;Read Lock Bits                   r                    dd
;Read Program Memory              R                    2*dd
;Read Signature Bytes             s                    3*dd
;Return Software Identifier       S                    s[7]
;Return Supported Device Codes    t                    n*dd     00
;Select Device Type               T        dd                   CR
;Return Software Version          V                    dd dd
;Set LED                          x        dd                   CR
;Clear LED                        y        dd                   CR
;Read Product ID                  h                    s[n]     CR

.include "C:\Program Files\Atmel\AVR Tools\AvrAssembler2\Appnotes\m8515def.inc"

;***********************************************************************************************
;* Register Definitions

;.def unused      = r0
;.def unused      = r1     
;.def unused      = r2 
;.def unused      = r3
;.def unused      = r4           ; 
;.def unused      = r5
;.def unused      = r6           ; 
;.def unused      = r9           ; 
;.def unused      = r10          ; 
;.def unused      = r11          ; 
;.def unused      = r12
;.def unused      = r13
;.def unused      = r14          ; 
;.def unused      = r15          ; 
 
.def temp        = r16          ; general purpose temporary register
.def temp2       = r17          ; general purpose temporary register
.def read_write  = r18          ; read_write == 6 then Write, if read_write == 1 then Read
;.def unused      = r19          ; 
;.def unused      = r20          ; 
;.def unused      = r21          ; 
.def data_lo     = r22          ; data low byte
.def data_hi     = r23          ; data high byte
;.def unused      = r25          ;
.def spm_action  = r24          ; determine SPM action

.undef XL
.undef XH
.def address_lo  = r26          ;  (XL) address low byte
.def address_hi  = r27          ;  (XH) address high byte
;.def YL          = r28          ; 
;.def YH          = r29          ;
;.def ZL          = r30          ;  
;.def ZH          = r31          ; 

;***********************************************************************************************
; Constants
.equ DT  = $3B   ; Device Type = 0x6A (ATmega8535), $3B = ATmega8515 (AVRProg only seems to recognise this)
.equ SB1 = $1E   ; Signature byte 1
.equ SB2 = $93   ; Signature byte 2
.equ SB3 = $08   ; Signature byte 3
.equ UBR = 25     ; Baud rate = 19.200 bps with fCK = 8.00 MHz

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
.equ  TONE           = 0            ; output - alarm tone or audio feedback
;.equ usused          = 1            ; Output - unused outputs
.equ PROGRAM         = 2            ; Input - enter program mode at power up
.equ LIGHT_SENSE     = 3            ; Input - light sensor input comparator
;.equ RES1            = 4            ; Output - resistor pulldown
;.equ RES1            = 5            ; Output - resistor pulldown
;.equ RES1            = 6            ; Output - resistor pulldown
.equ LED             = 7            ; Output - LED output
                          
.equ PORTB_DIR  = 0b11110011
.equ PORTB_INIT = 0b00000100

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
 
.equ PORTD_DIR  = 0b00010010 
.equ PORTD_INIT = 0b11100000      ; POWER off at start, trig pullup enabled 

;***********************************************************************************************
;* Start
.org THIRDBOOTSTART              ;($0E00) third boot block size is 512W
   nop   ; dummy nop incase the chip is blank, blank chip is programmed with all $FF
; $FF opcode and data corresponds to SBRS, $FF and will skip till boot section

   cli                           ; do not allow interrupts during programming

; write boot loader lock bits
   ldi   temp, 0b11101111        ; (1 << BLB11)        
   mov   R0, temp                ; SPM disabled in boot section
   ldi   temp, (1 << BLBSET) + (1 << SPMEN) ;  enable write of lock bits
   out   SPMCR, temp
   spm

   ldi   temp, PORTA_INIT        ; set initial values for PORTA
   out   PORTA, temp
   ldi   temp, PORTB_INIT        ; set initial values for PORTB
   out   PORTB, temp
   ldi   temp, PORTC_INIT        ; set initial values for PORTC
   out   PORTC, temp
   ldi   temp, PORTD_INIT        ; set initial values for PORTD
   out   PORTD, temp
   ldi   temp, PORTA_DIR         ; Initialize PORTA  pins direction.
   out   DDRA, temp           
   ldi   temp, PORTB_DIR         ; Initialize PORTB  pins direction.
   out   DDRB, temp            
   ldi   temp, PORTC_DIR         ; Initialize PORTC  pins direction.
   out   DDRC, temp          
   ldi   temp, PORTD_DIR         ; Initialize PORTD  pins direction.
   out   DDRD, temp

   sbic  PINB, PROGRAM          ; Skip next instruction if PINB3 cleared
   rjmp  FLASHEND+1             ; else normal execution from Reset (FLASHEND+1 = Address 0000)

   cbi   portb, LED             ; turn LED on

; Programming mode
   ldi   temp, low(RAMEND)
   ldi   temp2, high(RAMEND)
   out   SPL, temp
   out   SPH, temp2              ; SP = RAMEND
   ldi   temp, UBR               ; Baud rate = 19.200 bps
   out   UBRRL, temp
   ldi   temp, 0
   out   UBRRH, temp
   ldi   temp,(1<<RXEN)|(1<<TXEN)
   out   UCSRB, temp             ; Enable receiver & transmitter, 8-bit mode

   ldi   temp, 'P'               ; else uartSend('P') successful program mode
   rcall uartSend                ; uartSend(temp)
   ldi   temp, 13                ; uartSend('\r') 
   rcall uartSend                ; uartSend(temp)

main: 
   rcall uartGet                 ; repeat (temp = uartGet)
   cpi   temp, 27                ; while (temp == ESCAPE)
   breq  main
   cpi   temp, 'a'               ; if(temp=='a') 'a' = Autoincrement?
   brne  auto_inc
   ldi   temp, 'Y'               ; Yes, autoincrement is quicker
   rjmp  do_uart_send            ; uartSend(temp)

auto_inc: 
   cpi   temp, 'A'               ; else if(temp=='A') write address
   brne  write_prog_lo
   rcall uartGet
   mov   address_hi, temp        ; address high byte
   rcall uartGet
   mov   address_lo, temp        ; address low byte
   lsl   address_lo              ; address=address<<1
   rol   address_hi              ; convert from byte address to word address
   rjmp  carriage_return         ; uartSend('\r')

write_prog_lo: 
   cpi   temp, 'c'               ; else if(temp=='c') write program memory, low byte
   brne  write_prog_hi
   rcall uartGet
   mov   data_lo, temp           ; data_lo = data low byte
   rjmp  carriage_return         ; uartSend('\r')

write_prog_hi: 
   cpi   temp, 'C'               ; else if(temp=='C') write program memory, high byte
   brne  chip_erase
   rcall uartGet
   mov   data_hi, temp           ; data_hi = data high byte
   movw  ZL, address_lo          ; Z pointer = address
   movw  R0, data_lo             ; R0&R1 = data
   ldi   spm_action, (1 << SPMEN)
   rcall do_spm
   adiw  address_lo, 2           ; address=address+2
   rjmp  carriage_return         ; uartSend('\r')

chip_erase: 
   cpi   temp, 'e'               ; else if(temp=='e') Chip erase
   brne  write_page
; for(address=0; address < (2*THIRDBOOTSTART); address += (2*PAGESIZE))
   clr   address_lo              ; page_erase()
   clr   address_hi
   rjmp  chipe_2

chipe_1: 
   movw  ZL, address_lo          ; Z-pointer = address
   ldi   spm_action, (1 << PGERS) + (1 << SPMEN)

   rcall do_spm
   nop
   subi  address_lo, low(-2*PAGESIZE) ; address += (2*PAGESIZE)
   sbci  address_hi, high(-2*PAGESIZE)

chipe_2: 
   ldi   temp, low(2*THIRDBOOTSTART)
   ldi   temp2, high(2*THIRDBOOTSTART)
   cp    address_lo, temp        ; address < Boot Flash address(byte address) 0x3E00 ?
   cpc   address_hi, temp2
   brlo  chipe_1
; re-enable the RWW section
   ldi   spm_action, (1 << RWWSRE) + (1 << SPMEN)
   rcall do_spm
   rjmp  carriage_return         ; uartSend('\r')

write_page: 
   cpi   temp, 'm'               ; else if(temp== 'm') Write page
   brne  else_other
   movw  ZL, address_lo          ; Z-pointer = address
   ldi   spm_action, (1 << PGWRT) + (1 << SPMEN) 
   rcall do_spm
   nop
; re-enable the RWW section
   ldi   spm_action, (1 << RWWSRE) + (1 << SPMEN)
   rcall do_spm
   rjmp  carriage_return         ; uartSend('\r')

else_other:
   cpi   temp, 'E'               ; else if(temp=='E') Exit programming mode
   breq  exit_prog               ; uartSend('\r') 
   cpi   temp, 'P'               ; else if(temp=='P') Enter programming mode
   breq  enter_prog              ; uartSend('\r')
   cpi   temp, 'L'               ; else if(temp=='L') Leave programming mode
   breq  enter_prog              ; uartSend('\r')
   cpi   temp, 'p'               ; else if (temp=='p') Return programmer type
   brne  prog_type
   ldi   temp, 'S'               ; uartSend('S') Serial
   rjmp  do_uart_send            ; uartSend(temp)

enter_prog: 
   rjmp  carriage_return         ; uartSend('\r')

exit_prog:
   ldi   temp, (1 << TXC)        ; clear transmit complete flag
   out   UCSRA, temp
   ldi   temp, 13                ; uartSend('\r') 
   rcall uartSend                ; uartSend(temp)
ep_0:
   sbis  UCSRA, TXC             ; wait for transmit to complete
   rjmp  ep_0
   rjmp  FLASHEND+1

prog_type: 
   cpi   temp, 'R'               ; else if(temp=='R') Read program memory
   brne  write_data
   movw  ZL, address_lo          ; Z-pointer <= address
   lpm   spm_action, Z+          ; read program memory LSB; store LSB in spm_action and Z pointer ++
   lpm   temp, Z+                ; read program memory MSB; store MSB in temp and Z pointer ++
   rcall uartSend                ; uartSend(temp) MSB
   movw  address_lo, ZL          ; address += 2
   mov   temp, spm_action        ; LSB stored in temp
   rjmp  do_uart_send            ; uartSend(temp) LSB

write_data: 
   cpi   temp, 'D'               ; else if (temp=='D') Write data to EEPROM
   brne  read_data
   rcall uartGet
   out   EEDR, temp              ; EEDR = uartGet()
   ldi   read_write, 6           ; write EEPROM
   rcall EepromTalk
   rjmp  carriage_return         ; uartSend('\r')

read_data: 
   cpi   temp, 'd'               ; else if (temp=='d') Read data from EEPROM
   brne  read_fuse
   ldi   read_write, 1           ; read EEPROM
   rcall EepromTalk              ; temp = EEPROM data
   rjmp  do_uart_send            ; uartSend(temp)

read_fuse: 
   cpi   temp, 'F'               ; else if(temp=='F') Read fuse bits
   brne  read_lock
   clr   ZL                      ; Z-pointer = 0000
   rjmp  read_fuse_lock          ; rcall readFuseAndLock

read_lock: 
   cpi   temp, 'r'               ; else if(temp=='r') Read lock bits
   brne  read_fuse_hi
   ldi   ZL, 1                   ; Z pointer = 0001
   rjmp  read_fuse_lock          ; rcall readFuseAndLock

read_fuse_hi: 
   cpi   temp, 'N'               ; else if(temp=='N') Read high fuse bits
   brne  device_code
   ldi   ZL, 3                   ; Z-pointer = 0003

read_fuse_lock: 
   rcall readFuseAndLock
   rjmp  do_uart_send            ; uartSend(temp)

device_code: 
   cpi   temp, 't'               ; else if(temp=='t') Return supported devices code
   brne  else_do_1
   ldi   temp, DT                ; Device Type
   rcall uartSend                ; uartSend(DT) send Device Type
   clr   temp
   rjmp  do_uart_send            ; uartSend(0)

else_do_1:                       ; else if ((temp=='l')||(temp=='x')||(temp=='y')||(temp=='T'))
   cpi   temp, 'l'               ; 'l' = Write Boot Loader lockbits
   breq  do_uart_get
   cpi   temp, 'x'               ; 'x' = Set LED
   breq  LED_ON
   cpi   temp, 'y'               ; 'y' = Clear LED
   breq  LED_OFF
   cpi   temp, 'T'               ; 'T' = Select device type
   brne  send_soft_id

do_uart_get: 
   rcall uartGet                 ; temp = uartGet()
   rjmp  carriage_return         ; uartSend('\r')

LED_OFF:
   rcall uartGet                 ; temp = uartGet()
   sbi   portb, LED
   rjmp  carriage_return         ; uartSend('\r')

LED_ON:
   rcall uartGet                 ; temp = uartGet()
   cbi   portb, LED
   rjmp  carriage_return         ; uartSend('\r')   

send_soft_id: 
   cpi   temp, 'S'               ; else if (temp=='S') Return software identifier
   brne  Send_Prod
   ldi   ZL, low(2*Soft_Id)
   ldi   ZH, high(2*Soft_Id)

send_string: 
   lpm   temp, Z+
   tst   temp
   breq  end_of_string           ; branch is end of string ((Z) == 0)
   rcall uartSend                ; else send char
   rjmp  send_string

send_prod: 
   cpi   temp, 'h'               ; else if (temp=='h') Return product ID
   brne  soft_version
   ldi   ZL, low(2*Prod_Id)
   ldi   ZH, high(2*Prod_Id)
sp2:
   lpm   temp, Z+
   tst   temp
   breq  PC+3                    ; branch is end of string ((Z) == 0)
   rcall uartSend                ; else send char
   rjmp  sp2
   rjmp  carriage_return

soft_version: 
   cpi   temp, 'V'               ; else if (temp=='V') Return Software Version
   brne  signature_byte
   ldi   temp, '1'               ; uartSend('1')
   rcall uartSend
   ldi   temp, '0'               ; uartSend('0')
   rjmp  do_uart_send            ; uartSend(temp)

signature_byte: 
   cpi   temp, 's'               ; else if (temp=='s') Return Signature Byte
   brne  what
   ldi   temp, SB1               ; uartSend(SB1) Signature Byte 1
   rcall uartSend
   ldi   temp, SB2               ; uartSend(SB2) Signature Byte 2
   rcall uartSend
   ldi   temp, SB3               ; uartSend(SB3) Signature Byte 3
   rjmp  do_uart_send            ; uartSend(temp)

what: 
   ldi   temp, '?'               ; else uartSend('?')
   rjmp  do_uart_send            ; uartSend(temp)

carriage_return: 
   ldi   temp, 13                ; uartSend('\r')

do_uart_send: 
   rcall uartSend                ; uartSend(temp)

end_of_string: 
   rjmp  main

readFuseAndLock:
   clr   ZH                      ; Z pointer high byte = 0
   ldi   spm_action, 9           ; SPMCR = 0x09
   out   SPMCR, spm_action       ; read fuse and lock
   lpm   temp, Z                 ; read program memory
   ret

EepromTalk:                      ; if read_write == 6 then Write, if read_write == 1 then Read
   out   EEARL, address_lo       ; EEARL = address low
   out   EEARH, address_hi       ; EEARH = address high
   adiw  address_lo, 1           ; address++
   sbrc  read_write, 1           ; skip if read_write == 1 (read Eeprom)
   sbi   EECR, EEMWE             ; EEMWE = 1 (write Eeprom)
   out   EECR, read_write        ; EECR = read_write (6 write, 1 read)

wait_ee_1: 
   sbic  EECR, EEWE              ; wait until EEWE == 0
   rjmp  wait_ee_1
   in    temp, EEDR              ; temp = EEDR
   ret

do_spm:
; check for previous spm complete
wait_spm:
   in    temp, SPMCR
   sbrc  temp, SPMEN
   rjmp  wait_spm
; disable interrupts if enabled, store status
   in    temp, SREG
   cli   
; check that no EEPROM write access is going on
wait_ee:
   sbic  EECR, EEWE
   rjmp  wait_ee
; SPM timed sequence
   out   SPMCR, spm_action
   spm
; restore SREG (to enable interrupts if originally enabled)
   out   SREG, temp
   ret

uartSend:                        ; send temp
   sbis  UCSRA, UDRE             ; wait for empty transmit buffer (until UDRE==1)
   rjmp  uartSend
   out   UDR, temp               ; UDR = temp, start transmission
   ret

uartGet:
   sbis  UCSRA, RXC              ; wait for incoming data (until RXC==1)
   rjmp  uartGet
   in    temp, UDR               ; return received data in temp
   ret

Soft_Id: .DB "AVRBOOT", 0
Prod_Id: .DB "Nixie Clock 2", 0

; END of BOOT LOADER PROGRAM

