#!python
import os.path

env = SConscript('extern/laks/build/env.py')
#env.SelectMCU('gd32vf103cb')
#env.SelectMCU('ch32v307vct6')
env.SelectMCU('ch582m')

#env.Append(CPPDEFINES = ["CH58x_BOARD_DEV"])
env.Append(CPPDEFINES = ["CH58x_BOARD_VEITTUR"])

env.SetOption("num_jobs", 8) # TODO - get this from the system

env.Append(
	CXXFLAGS = Split('-fcoroutines -Wno-volatile'),
	LINKFLAGS = Split('--specs=nano.specs'),
        LIBS = ["ch58xble.a"],
        LIBPATH = "/home/karlp/src/wch-ch583.git/EVT/EXAM/BLE/LIB",
)

env.Append(CPPPATH = "/home/karlp/src/wch-ch583.git/EVT/EXAM/BLE/LIB/")

#fw = env.Program("miniblink1.elf", [os.path.join("src", x) for x in ["miniblink1.cpp"] + sources_shared] + sources_freertos + env['LIB_SOURCES'])
#fw = env.Program("miniblink1.elf", ["src/miniblink1.cpp"] )#+ env['LIB_SOURCES'])
#env.Depends(fw, env['LINK_SCRIPT'])

#env.Firmware('ble1.elf', [os.path.join('src', x) for x in ['ble1.cpp', 'syszyp.cpp']])
env.Firmware('ble1.elf', [os.path.join('src', x) for x in ['ble1.cpp', 'syszyp.cpp', 'broadcaster.c', 'devinfoservice.c']])


