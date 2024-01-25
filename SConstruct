#!python
import os.path

env = SConscript('extern/laks/build/env.py')
#env.SelectMCU('gd32vf103cb')
#env.SelectMCU('ch32v307vct6')
env.SelectMCU('k70fn1m0vmj12')

# Gross hack to make cmsis device includes work.  TODO - optionally add in selectMCU?
#env.Append(CPPDEFINES = ['STM32WB55xx'])

#env.Append(CPPDEFINES = ["CH582_DEV_BOARD"])
env.Append(CPPDEFINES = ["TWRK70"])
#env.Append(CPPDEFINES = ["K_VEITTUR"])

env.SetOption("num_jobs", 8) # TODO - get this from the system

env.Append(
	CXXFLAGS = Split('-fcoroutines -Wno-volatile'),
	LINKFLAGS = Split('--specs=nano.specs'),
)

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
sources_freertos += ["${FREERTOS}/portable/MemMang/heap_1.c"]

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
env.Firmware('kminiblink.elf', [os.path.join('src', x) for x in ['kminiblink1.cpp', 'plain_main.c', 'syszyp.cpp']])
#env.Firmware('spi-slave.elf', [os.path.join('src', x) for x in ['spi-slave.cpp', 'syszyp.cpp']])
#env.Firmware('timer-piezo1.elf', [os.path.join('src', x) for x in ['timer-piezo1.cpp', 'syszyp.cpp']])

