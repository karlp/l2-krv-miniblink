#!python
import os.path

env = SConscript('extern/laks/build/env.py')
#env.SelectMCU('gd32vf103cb')
#env.SelectMCU('ch32v307vct6')
env.SelectMCU('k70fn1m0vmj12')

# Gross hack to make cmsis device includes work.  TODO - optionally add in selectMCU?
#env.Append(CPPDEFINES = ['STM32WB55xx'])

#env.Append(CPPDEFINES = ["CH582_DEV_BOARD"])
env.Append(CPPDEFINES = ["TWRK70", "TWR_K70F120M",
        "CFG_TUSB_MCU=OPT_MCU_KINETIS_K32",
        "CPU_MK64FN1M0VLL12", # lol
])
#env.Append(CPPDEFINES = ["K_VEITTUR"])

env.SetOption("num_jobs", 8) # TODO - get this from the system

env.Append(
	CXXFLAGS = Split('-fcoroutines -Wno-volatile'),
	LINKFLAGS = Split('--specs=nano.specs'),
)

# env.Append(
#     LINKFLAGS = Split('-Wl,--print-gc-sections  -Wl,--print-map-discarded')
# )

freertos_arch = {
	"cortex-m7f": "ARM_CM7/r0p1",
	"cortex-m4f": "ARM_CM4F",
	"cortex-m3": "ARM_CM3",
	"cortex-m0": "ARM_CM0",
	"cortex-m0+": "ARM_CM0",
	"rv32imac": "RISC-V",
}

env.SetDefault(
        FREERTOS = "#extern/freertos",
        FREERTOS_PORT = "#extern/freertos/portable/GCC/%s" % freertos_arch.get(env["PLATFORM_SPEC"]["meta"]["cpu"], "UNKNOWN_FREERTOS_ARCH"),
        )

env.Append(
	CPPPATH = [
            "${FREERTOS}/include",
            "${FREERTOS_PORT}",
            "#src",
            ],
)

sources_freertos = [os.path.join("${FREERTOS}/", x) for x in Split("list.c queue.c tasks.c timers.c")]
sources_freertos += ["${FREERTOS_PORT}/port.c"]
sources_freertos += ["${FREERTOS}/portable/MemMang/heap_4.c"]

# ETL installation
env.Append(CPPPATH=["#extern/etl/include"])

# Uncomment if desired...
sources_shared = []
#sources_shared += ['call-graphing.cpp']
#env.Append(CCFLAGS = ["-finstrument-functions", "-finstrument-functions-exclude-file-list=cmsis,freertos,laks"])

#fw = env.Program("miniblink1.elf", [os.path.join("src", x) for x in ["miniblink1.cpp"] + sources_shared] + sources_freertos + env['LIB_SOURCES'])
#fw = env.Program("miniblink1.elf", ["src/miniblink1.cpp"] )#+ env['LIB_SOURCES'])
#env.Depends(fw, env['LINK_SCRIPT'])

#env.Firmware('miniblink1.elf', [os.path.join('src', x) for x in ['miniblink1.cpp', 'syszyp.cpp']])
#env.Firmware('kminiblink.elf', [os.path.join('src', x) for x in ['kminiblink1.cpp', 'plain_main.c', 'syszyp.cpp', "stdio-itm.cpp"]])
#env.Firmware('spi-slave.elf', [os.path.join('src', x) for x in ['spi-slave.cpp', 'syszyp.cpp']])
#env.Firmware('timer-piezo1.elf', [os.path.join('src', x) for x in ['timer-piezo1.cpp', 'syszyp.cpp']])


# we're being gross here and not using any of scons's layered stuff, just hard hammering in here...
# FIXME - so so gross, corrupting CPPPATH as welll...
# usbfshost_files = []
# usbfshost_files += Glob("USBFS_HOST/bsp/*.c")
# usbfshost_files += Glob("USBFS_HOST/classes/*/*.c")
# usbfshost_files += Glob("USBFS_HOST/common/*.c")
# usbfshost_files += Glob("USBFS_HOST/driver/*.c")
# usbfshost_files += Glob("USBFS_HOST/host_common/*.c")
# usbfshost_files += Glob("USBFS_HOST/*.c")
# usbfshost_files += Glob("k70/*.c")
# env.Append(
#     CPPPATH =[
#         "USBFS_HOST/common",
#         "USBFS_HOST/bsp",
#         "USBFS_HOST/host_common",
#         "USBFS_HOST/driver",
#         "USBFS_HOST/classes/common",
#         "USBFS_HOST/classes/hid",
#         "USBFS_HOST/classes/hub",
#         ]
# )

# env.Firmware("kusbfshost.elf", [os.path.join("src", x) for x in ["kusbfs_host_main.cpp", "syszyp.cpp", "stdio-itm.cpp"]] + usbfshost_files)

# usbfsdev_files = []
# usbfsdev_files += Glob("USBFS_device/*.c")
# usbfsdev_files += Glob("k70/*.c")
# env.Append(
#     CPPPATH =[
#         "USBFS_device",
#         "k70",
#         ]
# )

#env.Firmware("kusbfsdev.elf", [os.path.join("src", x) for x in ["kusbfs_dev_main.cpp", "syszyp.cpp", "stdio-itm.cpp"]] + usbfsdev_files)

tu_dev_files = Glob("extern/tinyusb/src/*.c")
tu_dev_files += Glob("extern/tinyusb/src/class/*/*.c")
tu_dev_files += Glob("extern/tinyusb/src/common/*.c")
tu_dev_files += Glob("extern/tinyusb/src/device/*.c")
#tu_dev_files += Glob("extern/tinyusb/src/portable/chipidea/ci_fs/*.c")
tu_dev_files += Glob("extern/tinyusb/src/portable/nxp/khci/*.c")

tu_dev_files += Glob("k70/*.c")
tu_dev_files += Glob("USBFS_device/bsp*.c")
tu_dev_files += Glob("extern/tinyusb/examples/device/hid_composite/src/*.c")
env.Append(CPPPATH=[
        "k70",
        "USBFS_device",
        "extern/tinyusb/hw",
        "extern/tinyusb/src",
        #"extern/tinyusb/src/common",
        #"extern/tinyusb/src/classes/hid",
        "extern/tinyusb/examples/device/hid_composite/src",
        "extern/tinyusb/hw/mcu/nxp/mcux-sdk/devices/MK64F12/",
        "extern/tinyusb/lib/CMSIS_5/CMSIS/Core/Include",

])

env.Firmware("tiny1.elf", [os.path.join("src", x) for x in ["tiny1dev.cpp", "tiny1dev_board.c", "syszyp.cpp", "stdio-itm.cpp"]] + tu_dev_files)
