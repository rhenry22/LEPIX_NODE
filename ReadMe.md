# STM32F407VET6 ChaDeMo / CCS2 and FoxESS Inverter Controller
There are 2 versions of the hardware: CCS2 and ChaDeMo.

The CCS2 version adds a CCS2 CP signal PWM and measurement HW block.
The build can be switched by setting the appropriate TARGET in the Makefile.

## Compatibility / Testing
Inverter: Fox ESS H1-AC-5.0 inverter (gen1) - works great!<br/>
ChaDeMo: 2015 Nissan Leaf<br/>
CCS2: Cupra Born (Car aborts after 1 min, but PLC comms and precharge works!)<br/>

## Serial Commands (Used from pyPLC / MQTT)
```
reset           : Reset the Controller

flash           : Put the Controller into DFU mode
espbridge       : Put the Controller into esptool compatible bridge mode for flashing esp8266
                  (requires power cycle to exit)

power           : Set the target power (+/- : Discharge / Charge)

hv              : Control the HV test generator
  iso <voltage>     : Set the isolation test voltage (0-500V)
  get               : Triggers a JSON update to measure the HV voltages and HV source current draw

evse            : Control the EVSE (Sink) and CCS2 (Source) parameters
  chg-en 0/1        : Disable / Enable the EVSE CP Line
  pwm 0-100         : Set the CCS2 CP PWM Value
  get               : Trigger a JSON update

led
  user <mask> <set> [flash]
  debug <mask> <set> [flash]

solax           : Solax / FoxESS Inverter Settings
  dc_max_i <current>  : Set the maximum charge and discharge current in A x10
  dc_max_v <voltage>  : Set the maximum battery voltage in V x10
  dc_min_v <voltage>  : Set the minimum battery voltage in V x10
  dc_tgt_v <voltage>  : Set the target battery voltage in V x10
  soc <SoC>           : Set the battery SoC in %
  enable <enable>     : Enable / Disable the Solax BMS emulation
```
## JSON
```
typedef enum {
  PP_NONE,     /* Plug not inserted */
  PP_PRESSED,  /* Plug inserted, button pressed */
  PP_INSERTED, /* Plug inserted, not pressed */
  PP_ERROR     /* Invalid reading */
} EVSE_PP;
```

```
typedef enum {
  CP_A,        /* No vehicle connected */
  CP_B,        /* Vehicle Connected, not ready */
  CP_C,        /* Vehicle Connected, Charge */
  CP_D,        /* Vehicle Connected, Charge (with ventilation) */
  CP_ERROR     /* Invalid reading */
} CCS2_CP;
```

```
typedef enum _solax_state {
  SOLAX_BATTERY_ANNOUNCE,
  SOLAX_REQUEST_CONTACTOR_CLOSE,
  SOLAX_CONTACTOR_PRECHARGE,
  SOLAX_CONTACTOR_CLOSED,
  SOLAX_FAULT,
  SOLAX_UPDATING_FW
} SOLAX_STATE;
```

```
typedef enum _chademo_state
{
  CHADEMO_STATE_OFF,
  CHADEMO_STATE_START,
  CHADEMO_STATE_PARAM_CHK,
  CHADEMO_STATE_PERM_OK,
  CHADEMO_STATE_INS_TEST_BASE,
  CHADEMO_STATE_INS_TEST,
  CHADEMO_STATE_BATT_CHECK,
  CHADEMO_STATE_ON,
  CHADEMO_STATE_STOP,
  CHADEMO_STATE_WELD_CHECK,
  CHADEMO_STATE_WAIT_K_OFF,
  CHADEMO_STATE_WAIT_VEHICLE_OFF,
  CHADEMO_STATE_ERROR
} CHADEMO_STATE;
```

## Controller JSON Output
```
{"controller":{"power_offset":<power in W>,"timestamp":<tick ms>,
    "sensors":{"acc":{"v":<mV>, "i":<uA>>},"hv_iso":{"i":<uA>>, "r":<kOhm>},"battery":{"v":<V x10>, "i":<A x10>},"inverter":{"v":<V x10>, "i":<A x10>}}
{"evse":{"ac":{"max_current":<A>>,"pp":<EVSE_PP>, "ccs2":{"cp":<CCS2_CP>, "pwm":<ccs2_pwm>}}}}
{"solax":{"state":<SOLAX_STATE>, "last_error":<string>, "inv_state":<raw state>, "inv_temp":<Cx10>, "grid_power":<W>, "inv_fault":[<faut array>]}}
{"chademo":{"state":<CHADEMO_STATE>, "cp_ready":<car ready>, "voltage":<V>, "current":<A>, "power":<W>, "last_error":<string>}}
{"controller":[{"cmd":"<cmd executed>","status":"<return code>"}]}
```
## Building
Install the arm-none-eabi tools.
```
apt install gcc-arm-none-eabi <br/>
make
```
## Flashing
The STM32F407 has built in DFU functionality.<br/>
Move the BOOT0 jumper from '0' to '1', and connect the mini USB connection to a PC.
```
make flash
```
Now move the BOOT0 jumper back to '0' and hit reset / power cycle the board.

# ESP8266

## Tasmota Flashing
Put the board into esp bridge mode - this will expose the ESP8266 over USB to enable esptool / tasmotizer to be used.

Connect to the USB port
Send "espbridge" with a <CR>

The board will reset and enter bridge mode - you must power cycle to exit.

## Tasmota Setup
After flashing Tasmota, the ESP8266 and STM32 will battle each other by failing to understand what each other are saying!
Ignore this, and configure Tasmota from the Web UI:

### Configuration -> Logging
Serial Log Level: 0 (None)

### Configuration -> Module
Generic (0)
TX GPIO1 as SerBr TX
RX GPIO3 as SerBr RX

### Console
Automatically set the serial port up and boot, and use \n as the delimiter.
```
Rule1 ON System#Boot DO Backlog Baudrate 115200; SerialLog 0 ENDON
Rule1 1
SerialDelimiter 10
```

### MQTT
After setting up the MQTT server, you should see the module output coming in via SSerialReceived JSON messages.

# Schematics
[Custom Control Board Schematics](Docs/v2x_adaptor.pdf)

# Original Control Panel (converted to CCS2)
This is the original (rev.1) panel, converted to CCS2
![Image of ChaDeMo Control Panel](Docs/Original%20Control%20Panel.jpg)

# Mini ChaDeMo Control Panel
This is a smaller version (rev.2) of my original panel
![Image of ChaDeMo Control Panel](Docs/Mini%20ChaDeMo%20Control%20Panel.jpg)
