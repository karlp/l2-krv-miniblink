#!python
import collections
import os.path


Board = collections.namedtuple("Board", "brd part led1 led1_enable tu_mcu mcuxinc")
boards_kx = [
    # TODO enable these other badbois
    Board("TWR-K70F120M", "mk70fn1m0vmj12", "GPIOA[11]", "sim::PORTA", "OPT_MCU_KINETIS_K", "MK70F12"), # orange led
 #   Board("FRDM-K66", "mk66fn2m0vmd18", "GPIOA[11]", "sim::PORTA"), # Blue led on RGB
    Board("FRDM-K64", "MK64FN1M0VLL12", "GPIOB[21]", "sim::PORTB", "OPT_MCU_KINETIS_K", "MK64F12"), # Blue led on RGB
]

# Add __NVIC_PRIO_BITS and a systemcoreclock? to board vars?

freertos_arch = {
	"cortex-m7f": "ARM_CM7/r0p1",
	"cortex-m4f": "ARM_CM4F",
	"cortex-m3": "ARM_CM3",
	"cortex-m0": "ARM_CM0",
	"cortex-m0+": "ARM_CM0",
	"rv32imac": "RISC-V",
}

for b in boards_kx:
    bdir = f"build/{b.part.lower()}"
    env = SConscript('extern/laks/build/env.py')
    env.SelectMCU(b.part, variant_dir=bdir)
    env.SetOption("num_jobs", 8)
    env.Append(CPPPATH=bdir)
    env.Append(CPPDEFINES = [
            ("BOARD", b.brd),
            ("PART", b.part),
            ("GPIO_LED1", b.led1),
        ])
    if b.led1_enable:
        env.Append(CPPDEFINES = [
            ("RCC_ENABLE1", b.led1_enable),
        ])


    # laks standard stuff.
    env.Append(
        CXXFLAGS = Split('-fcoroutines -Wno-volatile'),
        LINKFLAGS = Split('--specs=nano.specs'),
    )

    # FreeRTOS stuff
    env.SetDefault(
            FREERTOS = "#extern/freertos",
            FREERTOS_PORT = "#extern/freertos/portable/GCC/%s" % freertos_arch.get(env["PLATFORM_SPEC"]["meta"]["cpu"], "UNKNOWN_FREERTOS_ARCH"),
            )
    env.Append(
        CPPPATH = [
                "${FREERTOS}/include",
                "${FREERTOS_PORT}",
                #"#src", # This is "not freertos"!
                ],
    )
    fr_src = [os.path.join("${FREERTOS}/", x) for x in Split("list.c queue.c tasks.c timers.c")]
    fr_src += ["${FREERTOS_PORT}/port.c"]
    fr_src += ["${FREERTOS}/portable/MemMang/heap_4.c"]  # tinyusb doesn't use this!
    fr_objs = [env.Object(target=f"{bdir}/{f}", src=f"#{f}") for f in fr_src]

    # Right now, we're just sneakily grabbing it ahead of time out of the tusb repo, we know they have it.
    rtt_src = ["#extern/tinyusb/lib/SEGGER_RTT/RTT/SEGGER_RTT.c"]
    rtt_objs = [env.Object(target=f"{bdir}/{f}", src=f"#{f}") for f in rtt_src]
    env.Append(CPPPATH="#extern/tinyusb/lib/SEGGER_RTT/RTT")

    # woudl need to remove cpppath again, cant' clone the env as that makes dups for the laks files.. just comment it out
    env.Append(CPPPATH="#src")
    minib_objs = [env.Object(target=f"{bdir}/{f}.o", source=f"#src/{f}") for f in ["miniblink-freertos.cpp", "syszyp.cpp", "stdio-rtt.cpp"]]
    env.Firmware(f"miniblink-freertos-{b.brd}.elf", minib_objs + fr_objs + rtt_objs, variant_dir=bdir)


    # let's gooooo!
    env.SetDefault(TINYUSB="#extern/tinyusb")

    # Ok. here comes the bangers...
    tu_lib = []
    # I can't figure out how to make  this play properly with the variant dir, but it works fine with explicit names.
    # somethign to do with the ${TINYUSB} getting expanded or not, and how it detects where to copy shit.
    # tu_lib += env.Glob("${TINYUSB}/src/*.c", source=True)
    # tu_lib += env.Glob("${TINYUSB}/src/common/*.c", source=True)
    # tu_lib += env.Glob("${TINYUSB}/src/host/*.c", source=True)
    # tu_lib += env.Glob("${TINYUSB}/src/class/cdc/*.c", source=True)
    # tu_lib += env.Glob("${TINYUSB}/src/class/hid/*.c", source=True)
    # tu_lib += env.Glob("${TINYUSB}/src/class/hid/*.c", source=True)
    # tu_lib += env.Glob("${TINYUSB}/src/class/msc/*.c", source=True)
    # tu_lib += env.Glob("${TINYUSB}/src/portable/nxp/khci/*.c", source=True)
    tu_lib += [
        '${TINYUSB}/src/tusb.c',
        '${TINYUSB}/src/common/tusb_fifo.c',
        '${TINYUSB}/src/host/hub.c',
        '${TINYUSB}/src/host/usbh.c', 
        '${TINYUSB}/src/class/cdc/cdc_device.c',
        '${TINYUSB}/src/class/cdc/cdc_host.c',
        '${TINYUSB}/src/class/cdc/cdc_rndis_host.c',
        '${TINYUSB}/src/class/hid/hid_device.c', 
        '${TINYUSB}/src/class/hid/hid_host.c', 
        '${TINYUSB}/src/class/msc/msc_device.c', 
        '${TINYUSB}/src/class/msc/msc_host.c', 
        '${TINYUSB}/src/portable/nxp/khci/dcd_khci.c',
        '${TINYUSB}/src/portable/nxp/khci/hcd_khci.c',
        #'${TINYUSB}/hw/bsp/board.c', # lets not, it wants to own ITM vs RTT vs UART
        ]
    tu_example = []
    tu_example += ["${TINYUSB}/examples/host/cdc_msc_hid_freertos/src/cdc_app.c"]
    tu_example += ["${TINYUSB}/examples/host/cdc_msc_hid_freertos/src/hid_app.c"]
    tu_example += ["${TINYUSB}/examples/host/cdc_msc_hid_freertos/src/msc_app.c"]
    tu_example += ["${TINYUSB}/examples/host/cdc_msc_hid_freertos/src/freertos_hook.c"]

    #print("ok, ", tu_lib[0], tu_example[0]  )
    
    tu_src = tu_lib + tu_example
    #print("wat?", tu_src)
    tu_objs = []
    #tu_objs = [env.Object(target=f"{bdir}/{f}", src=f"#{f}") for f in tu_src]
    print("yo, for reals, what's in our obj list?", [f[0].path for f in tu_objs])
    # ok, why doesn't it work for the globbed ones?
    #tu_objs += [env.Object(target=f"{bdir}/{f.path}", src=f"#{f.path}") for f in tu_lib]
    # env.Append(
    #     CPPPATH=[
    #         "${TINYUSB}/src",
    #         "${TINYUSB}/hw",
    #         # Remember,  python tools/get_deps.py kinetis_k first to make this work!
    #         #"${TINYUSB}/hw/mcu/nxp/mcux-sdk/devices/%s" % (b.mcuxinc), # lol, no!
    #         "src/mcux-stub",
    #         "${TINYUSB}/lib/CMSIS_5/CMSIS/Core/Include", # both tusb and mcux use cmsis heavily     
    #         "${TINYUSB}/examples/host/cdc_msc_hid_freertos/src",  # for tusb_config.h
    #         "src/tueh/cdc_msc_hid_freertos",

    #     ]
    # )

    env.Append(CPPDEFINES=[
        ("CFG_TUSB_MCU", b.tu_mcu),
        ("CFG_TUSB_DEBUG", 2),  # This is the LOG=n level in tinyusb make vars.
        f"CPU_{b.part.upper()}",
    ])
    #app_objs = [env.Object(target=f"{bdir}/{f}.o", source=f"#src/tueh/cdc_msc_hid_freertos/{f}") for f in ["main.cpp"]]
    #app_objs +=[env.Object(target=f"{bdir}/{f}.o", source=f"#src/{f}") for f in ["syszyp.cpp", "stdio-itm.cpp"]]
    # We're goign to get RTT going on the mini demo first!
    # env.Firmware(f"tue_h_cdc_msc_hid_freertos-{b.brd}.elf", tu_objs + app_objs + fr_objs)
    