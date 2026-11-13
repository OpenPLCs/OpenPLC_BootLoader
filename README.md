# OpenPLC_BootLoader

Bootloader for the OpenPLC board (STM32H743): it starts the user's sketch and accepts
signed uploads from Arduino IDE over USB or Ethernet.

Build it as an STM32CubeIDE project (`OpenPLC_BootLoader`); the output lands in `Debug/`.

User instructions, including how to flash this bootloader onto a board and how to upload
sketches from Arduino IDE, are in
[OpenPLC_Instruction](https://github.com/OpenPLCs/OpenPLC_Instruction/tree/master/docs/user).
Upgrade notes for this release are in [RELEASE-NOTES.md](RELEASE-NOTES.md).
