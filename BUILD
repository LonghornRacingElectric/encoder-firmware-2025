load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@aspect_bazel_lib//lib:transitions.bzl", "platform_transition_filegroup")

cc_binary (
    name = "encoder_firmware_2025",
    includes = ["Core/Inc", "Drivers/STM32L4xx_HAL_Driver/Inc", "Drivers/STM32L4xx_HAL_Driver/Inc/Legacy", "Drivers/CMSIS/Device/ST/STM32L4xx/Include", "Drivers/CMSIS/Include", "USB_DEVICE/App", "USB_DEVICE/Target", "Middlewares/ST/STM32_USB_Device_Library/Core/Inc", "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc", "Libs/longhorn-lib"],
    srcs = glob([
        "Core/**/*.c",
        "Core/**/*.h",
        "Drivers/**/*.c",
        "Drivers/**/*.h",
        "Middlewares/**/*.c",
        "Middlewares/**/*.h",
        "USB_DEVICE/**/*.c",
        "USB_DEVICE/**/*.h",
        # "Libs/**/*.c",
        # "Libs/**/*.h",
        ]),
    copts = [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mthumb-interwork",
        "-ffunction-sections",
        "-fdata-sections",
    ],
    deps = [
        "STM32L496RGTX_FLASH.ld"
    ],
    target_compatible_with = [
        "@platforms//cpu:arm",
        "@platforms//os:none",
    ],
    linkopts = [
        "-T $(execpath STM32L496RGTX_FLASH.ld)",
        "-Wl,--no-warn-rwx-segments",
    ],
    defines = ["DEBUG", "USE_HAL_DRIVER", "STM32L496xx",],
    additional_linker_inputs = ["STM32L496RGTX_FLASH.ld"]
)

platform(
    name = "arm_none_eabi",
    constraint_values = [
        "@platforms//cpu:arm",
        "@platforms//os:none",
    ],
)