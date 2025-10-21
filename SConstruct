#!python
import collections
import os.path


Board = collections.namedtuple("Board", "brd part led1 led1_enable led1_mux tu_mcu mcuxinc")
boards_kx = [
    # TODO enable these other badbois
    #Board("TWR-K70F120M", "mk70fn1m0vmj12", "GPIOA[11]", "sim::PORTA", "OPT_MCU_KINETIS_K", "MK70F12"), # orange led
 #   Board("FRDM-K66", "mk66fn2m0vmd18", "GPIOA[11]", "sim::PORTA", "PCRA"), # Blue led on RGB
    Board("FRDM-K64", "MK64FN1M0VLL12", "GPIOB[21]", "sim::PORTB", "PCRB", "OPT_MCU_KINETIS_K", "MK64F12"), # Blue led on RGB
    Board("V2400F", "MK70FN1M0VMJ15", "GPIOE[24]", "sim::PORTE", "PCRE", "OPT_MCU_KINETIS_K", "MK70F12"), # "activity"
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
            ("GPIO_LED1_MUX", b.led1_mux),
            ("HOST_ECHO", 1),
            ("SDK_DEBUGCONSOLE", 0),
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
    fr_src = [os.path.join("${FREERTOS}/", x) for x in Split("list.c queue.c tasks.c timers.c event_groups.c")]
    fr_src += ["${FREERTOS_PORT}/port.c"]
    fr_src += ["${FREERTOS}/portable/MemMang/heap_4.c"]  # tinyusb doesn't use this!
    fr_objs = [env.Object(target=f"{bdir}/{f}", src=f"#{f}") for f in fr_src]

    # Right now, we're just sneakily grabbing it ahead of time out of the tusb repo, we know they have it.
    rtt_src = ["#extern/tinyusb/lib/SEGGER_RTT/RTT/SEGGER_RTT.c"]
    rtt_objs = [env.Object(target=f"{bdir}/{f}", src=f"#{f}") for f in rtt_src]
    env.Append(CPPPATH="#extern/tinyusb/lib/SEGGER_RTT/RTT")
    env.Append(CPPDEFINES=[("LOGGER_RTT", 1)])

    meh_example = []
    meh_example += ["#src/meh/host2/app.c"]
    meh_example += ["#src/meh/host2/host_keyboard.c"]
    meh_example += ["#src/meh/host2/host_mouse.c"]

    env.SetDefault(MCUXU="#extern/mcux-usb")
    env.SetDefault(MCUXC="#extern/mcux-components")

    meh_lib = []
    meh_lib += [
        '${MCUXU}/host/usb_host_hci.c',
        '${MCUXU}/host/usb_host_khci.c',
        '${MCUXU}/host/usb_host_framework.c',
        '${MCUXU}/host/usb_host_devices.c',
        '${MCUXU}/host/class/usb_host_hub.c',
        '${MCUXU}/host/class/usb_host_hub_app.c',
        '${MCUXU}/host/class/usb_host_hid.c',
        '${MCUXC}/osa/fsl_os_abstraction_free_rtos.c',
        '${MCUXC}/lists/fsl_component_generic_list.c',
    ]
    meh_src = meh_lib + meh_example
    meh_objs = [env.Object(target=f"{bdir}/{f}", src=f"#{f}") for f in meh_src]
    env.Append(
        CPPPATH=[
            "src/mcux-stub",
            "${MCUXU}/host",
            "${MCUXU}/host/class",
            "${MCUXU}/include",
            "${MCUXC}/lists",
            "${MCUXC}/osa",
            "${MCUXC}/osa/config",
            "src/meh/host2",
            # FIXME - make this not depend on tusb!
            "#extern/tinyusb/lib/CMSIS_5/CMSIS/Core/Include", # both tusb and mcux use cmsis heavily
        ]
    )
    env.Append(CPPDEFINES=[
        "USB_STACK_FREERTOS",
        "SDK_OS_FREE_RTOS",
        ("USB_STACK_FREERTOS_HEAP_SIZE", 32768),
        ("FSL_OSA_BM_TASK_ENABLE", 0),
        ("FSL_OSA_BM_TIMER_CONFIG", 0),
        f"CPU_{b.part.upper()}",
    ])

    app_objs = [env.Object(target=f"{bdir}/{f}.o", source=f"#src/meh/host2/{f}") for f in ["main.cpp", "freertos-static-helpers.c"]]
    app_objs +=[env.Object(target=f"{bdir}/{f}.o", source=f"#src/{f}") for f in ["syszyp.cpp", "stdio-rtt.cpp"]]
    # app_objs +=[env.Object(target=f"{bdir}/{f}.o", source=f"#src/{f}") for f in ["syszyp.cpp"]]
    env.Firmware(f"meh_host2-{b.brd}.elf", meh_objs + app_objs + fr_objs + rtt_objs)
