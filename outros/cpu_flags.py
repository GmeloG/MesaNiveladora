# Força as flags do Cortex-M7 (STM32F767) em compilação e linkagem.
# Corrige o erro "Thumb-1 'hard-float' VFP ABI" / "Unknown Arm architecture
# profile" da plataforma ststm32 20.0.0, que não passava o -mcpu.
Import("env")

cpu = ["-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16", "-mfloat-abi=hard"]

env.Append(
    CCFLAGS=cpu,
    ASFLAGS=cpu,
    ASPPFLAGS=cpu,
    LINKFLAGS=cpu + ["-Wl,-u,_printf_float"],  # printf com floats (%.2f)
)
