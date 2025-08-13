load("@rules_cc//cc:defs.bzl", "cc_binary")
filegroup(
    name = "linker_script",
    srcs = ["STM32L496RGTX_FLASH.ld"],
)

filegroup(
    name = "startup_file",
    srcs = ["Core/Startup/startup_stm32l496rgtx.s"],
)
cc_binary(
    name = "encoder_firmware_2025",
    srcs = glob(
        [
            "Core/Src/**/*.c",
            "Drivers/**/*.c",
            "Middlewares/**/*.c",
            "USB_DEVICE/**/*.c",
            "Core/Inc/**/*.h",
        "Drivers/**/*.h",
        "Middlewares/**/*.h",
        "USB_DEVICE/**/*.h",
        ],

        exclude = ["Core/Startup/startup_stm32l496rgtx.s"],
    ),

    includes = [
        "Core/Inc",
        "Drivers/STM32L4xx_HAL_Driver/Inc",
        "Drivers/STM32L4xx_HAL_Driver/Inc/Legacy",
        "Drivers/CMSIS/Device/ST/STM32L4xx/Include",
        "Drivers/CMSIS/Include",
        "USB_DEVICE/App",
        "USB_DEVICE/Target",
        "Middlewares/ST/STM32_USB_Device_Library/Core/Inc",
        "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc",
        "Libs/longhorn-lib",
    ],

    # Compiler options
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mthumb-interwork",
        "-ffunction-sections",
        "-fdata-sections",
    ],

    # Linker options
    linkopts = [
        "-T $(location :linker_script)",
        "$(location :startup_file)",
        "-Wl,-Map=output.map",
        "-Wl,--gc-sections",
        "-Wl,--no-warn-rwx-segments",
    ],

    # Defines for the C preprocessor
    defines = [
        "DEBUG",
        "USE_HAL_DRIVER",
        "STM32L496xx",
    ],

    additional_linker_inputs = [
        ":linker_script",
        ":startup_file",
    ],

    # Target platform constraints
    target_compatible_with = [
        "@platforms//cpu:arm",
        "@platforms//os:none",
    ],
)

platform(
    name = "arm_none_eabi",
    constraint_values = [
        "@platforms//cpu:arm",
        "@platforms//os:none",
    ],
)

genrule(
    name = "bin",

    srcs = [":encoder_firmware_2025"],

    outs = ["encoder_firmware_2025.bin"],

    cmd = "$(execpath @arm_none_eabi//:objcopy) -O binary $< $@",
    cmd_bat = "$(execpath @arm_none_eabi//:objcopy) -O binary $< $@",
    # cmd_bat = "copy \"$(location @arm_none_eabi//:objcopy)\" && objcopy.exe -O ihex $< $@",

    tools = ["@arm_none_eabi//:objcopy"],
)



genrule(
    name = "hex",

    srcs = [":encoder_firmware_2025"],

    outs = ["encoder_firmware_2025.hex"],

    cmd = "$(execpath @arm_none_eabi//:objcopy) -O ihex $< $@",
    cmd_bat = "copy \"$(location @arm_none_eabi//:objcopy)\" && objcopy.exe -O ihex $< $@",

    tools = ["@arm_none_eabi//:objcopy"],
)