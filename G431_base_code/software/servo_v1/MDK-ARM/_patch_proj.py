#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把电流采样模块登记进 Keil 工程与 cproject：分组、源文件、Include 路径、HAL 库文件。"""
import os, sys

ROOT = r"C:\Users\penghao\Desktop\Servo_9948\G431_base_code"
SW = os.path.join(ROOT, "software", "servo_v1")
MDK = os.path.join(SW, "MDK-ARM")


def patch(path, pairs):
    with open(path, "rb") as f:
        raw = f.read()
    bom = b""
    body = raw
    if raw.startswith(b"\xef\xbb\xbf"):
        bom = b"\xef\xbb\xbf"
        body = raw[3:]
    crlf = b"\r\n" in body
    text = body.decode("utf-8")
    for old, new in pairs:
        if crlf:
            old = old.replace("\n", "\r\n")
            new = new.replace("\n", "\r\n")
        n = text.count(old)
        if n != 1:
            print("FAIL %s: 匹配 %d 次（需恰好 1 次）\n---\n%s\n---" % (
                os.path.basename(path), n, old[:300]))
            sys.exit(1)
        text = text.replace(old, new)
    out = bom + text.encode("utf-8")
    with open(path, "wb") as f:
        f.write(out)
    print("OK %-24s %6d -> %6d bytes  BOM=%-5s CRLF=%s" % (
        os.path.basename(path), len(raw), len(out), bool(bom), crlf))


# ------------------------------------------------------------ uvprojx: 分组
CS_GROUP_OLD = """        <Group>
          <GroupName>Modules/Current Sampling</GroupName>
        </Group>"""

CS_GROUP_NEW = """        <Group>
          <GroupName>Modules/Current Sampling</GroupName>
          <Files>
            <File>
              <FileName>adc1_current_sampling.c</FileName>
              <FileType>1</FileType>
              <FilePath>../Modules/Current_Sampling/Src/adc1_current_sampling.c</FilePath>
            </File>
            <File>
              <FileName>adc1_current_sampling.h</FileName>
              <FileType>5</FileType>
              <FilePath>../Modules/Current_Sampling/Inc/adc1_current_sampling.h</FilePath>
            </File>
          </Files>
        </Group>"""

# -------------------------------------------------------- uvprojx: HAL 库文件
HAL_OLD = """            <File>
              <FileName>stm32g4xx_hal_tim_ex.c</FileName>
              <FileType>1</FileType>
              <FilePath>../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c</FilePath>
            </File>
          </Files>"""

HAL_NEW = """            <File>
              <FileName>stm32g4xx_hal_tim_ex.c</FileName>
              <FileType>1</FileType>
              <FilePath>../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c</FilePath>
            </File>
            <File>
              <FileName>stm32g4xx_hal_adc.c</FileName>
              <FileType>1</FileType>
              <FilePath>../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c</FilePath>
            </File>
            <File>
              <FileName>stm32g4xx_hal_adc_ex.c</FileName>
              <FileType>1</FileType>
              <FilePath>../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c</FilePath>
            </File>
            <File>
              <FileName>stm32g4xx_hal_opamp.c</FileName>
              <FileType>1</FileType>
              <FilePath>../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp.c</FilePath>
            </File>
            <File>
              <FileName>stm32g4xx_hal_opamp_ex.c</FileName>
              <FileType>1</FileType>
              <FilePath>../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp_ex.c</FilePath>
            </File>
          </Files>"""

# --------------------------------------------------- uvprojx: Include 路径
INC_OLD = ("../Core/Inc;../FOC/Inc;../Modules/Driver_Communication/Inc;"
           "../Modules/Three_Phase_Driver/Inc;../Tests/Inc;")
INC_NEW = ("../Core/Inc;../FOC/Inc;../Modules/Driver_Communication/Inc;"
           "../Modules/Three_Phase_Driver/Inc;../Modules/Current_Sampling/Inc;../Tests/Inc;")

patch(os.path.join(MDK, "servo_v1.uvprojx"), [
    (CS_GROUP_OLD, CS_GROUP_NEW),
    (HAL_OLD, HAL_NEW),
    (INC_OLD, INC_NEW),
])

# ------------------------------------------------------------- cproject.yml
YML_PATH_OLD = """      add-path:
        - ../Core/Inc
        - ../FOC/Inc
        - ../Modules/Driver_Communication/Inc
        - ../Modules/Three_Phase_Driver/Inc
        - ../Tests/Inc"""
YML_PATH_NEW = """      add-path:
        - ../Core/Inc
        - ../FOC/Inc
        - ../Modules/Driver_Communication/Inc
        - ../Modules/Three_Phase_Driver/Inc
        - ../Modules/Current_Sampling/Inc
        - ../Tests/Inc"""

YML_GROUP_OLD = """    # Reserved functional groups for upcoming peripherals.
    - group: Modules/Current Sampling
      files: []
    - group: Modules/Encoder"""
YML_GROUP_NEW = """    - group: Modules/Current Sampling
      files:
        - file: ../Modules/Current_Sampling/Src/adc1_current_sampling.c
        - file: ../Modules/Current_Sampling/Inc/adc1_current_sampling.h
    # Reserved functional groups for upcoming peripherals.
    - group: Modules/Encoder"""

YML_HAL_OLD = """        - file: ../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    - group: Drivers/CMSIS"""
YML_HAL_NEW = """        - file: ../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
        - file: ../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
        - file: ../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
        - file: ../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp.c
        - file: ../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp_ex.c
    - group: Drivers/CMSIS"""

patch(os.path.join(MDK, "servo_v1.cproject.yml"), [
    (YML_PATH_OLD, YML_PATH_NEW),
    (YML_GROUP_OLD, YML_GROUP_NEW),
    (YML_HAL_OLD, YML_HAL_NEW),
])

print("工程登记完成")
