
Ok. CY7C65642 HX2VL usb hub works fine with linux _AS HIGH SPEED_

Would very much like to try that out with linux as _FULL_SPEED_
either need to get a FS only hub (can reprogram a hubbish for that!) _or_ need a working FS host with tinyusb...


## FS Host options
We _want_ to test the hub in FS mode, to rule out (hopefully) issues related to: https://github.com/hathach/tinyusb/issues/2517
The goal of testing this with linux is that we believe that linux's host stack is _far_ more robust and tested than tinyusb!
If the hub works in FS mode with linux, it's "just sw" to fix tinyusb! (and to hell with what olimex thinks!)

### hubbish from home
Need to bring in
* hubbish
* an esp32 board, so I can use that prebuilt i2c programmer example to reprogram it.

==> This is a good option!

### FRDM-K66 (hcd_ci_hs)
`ci_hs` is supported for host mode in tinyusb, but only for imxrt10xx and lpc18_43.  NXP MCX allegedly has the hardware, but no listed host support yet

However, K66 is not (yet) supported at all, as "kinetis_k" in tinyusb is a bit rigid...
it _should_ be feasible to add it to my laks treee, but, that would only get us, eventually, another (partially) tested hcd_ci_hs implementation.
hcd_ci_hs _is_ used on MWS41 though!  (but normally wired as a device!)

==> _probably_ not a lot of work, but... why? 

### stm32f429 nucleo (dwc_otg...)
This is widely supported, and probably pretty robust, but it doesn't get us anywhere?
might be worth a play?
XMC4800 uses dwc_otg too?

==> not a great option, but you have the hardware?

### K32L2 / K64 / K70

This is ~same situation as using the frdm-k66.  Except it's not the "supported" ci_hs, but the "not reallllly supported" khci/ci_fs in host mode.
Advantages are:
* both of these are nominally supported _boards_ so should be easier to get things upstream with them.
* we have both of them.

K64 is probably best path forward here.  proper cortex with ITM at least on a mini10pin header

K70-TWR is... legal, from a "this is a board you can upstream" but... it's not really, no-one has one, it's not in mcux, it's never going to work with upstream as an example, and it's unobtainium.
So, I can keep it in my tree where I build all the variants, but it's terrible for doing upstreamable work.



## Ok, so plan
* hubbish and an esp32 board to do i2c reprogramming into fs mode.
* test with linux
* assume it works  <<< 20240905 - yes, it works, see extra logs
* frdm-k64 base plan
* the board is in upstream!
* Get LS working robustly single port (ignore hub for now)
  (robust means plug/unplug, on boot, after boot, repeated)
 * all three keypads and mice are LS
* Get FS working robustly single port (ignore hub for now)
 * red barcode scanner is FS
 * usb memory key is HS (will fallback to FS eventually)
 * Can't use silabs/ch340 without including more host drivers!
* Add the hub in (no devices)
 * make plug/unplug work robustly....
* Probably starts working at this point?


## Ok, detailed steps for k64...
which host examples are going to be best....
Let's contine with "device-info" for now..

oh yeah, I should totally fucking cheat and run shit with mcux to compare and fix tusb right?

## with mcux mcuxsdk/examples/frdmk64f/usb_examples/usb_host_hid_mouse_keyboard/freertos/
with external blue trendnet hub, plug/unplug of keypad/mouse on any port works fine, robustly.

v2400f hub only works if attached _after_ startup. (and if it gets into a fucked up state, it needs to be power cycled!)
with v2400f hub, mouse always works in both ports.
