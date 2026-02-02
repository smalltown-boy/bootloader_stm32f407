# Bootloader for STM32F407VET6 (Project for STM32CubeIDE v1.19.0)

This bootloader is designed for firmware uploading via Ethernet (W5500).

## Connection Settings

- **IP Address:** `10.0.28.60`
- **Subnet Mask:** `255.255.0.0`
- **Port:** `5555`
- **Protocol:** UDP

## Usage Requirements

1. The PC and the target device must be on the same subnet.
2. The main firmware must be compiled as a `.bin` file and must not exceed **480 KB** (the bootloader occupies **32 KB** of Flash memory).
3. To activate the bootloader, pull pin **PB1** to ground.

## W5500 Wiring Diagram

| STM32F407VET6 | W5500 |
|--------------|-------|
| PB15         | MOSI  |
| PB14         | MISO  |
| PB13         | SCK   |
| PB12         | CS    |
| PD0          | RST   |

## Data Exchange

### Step 1. Sending Commands to the Bootloader

After startup, the bootloader accepts two commands:

- `0xAA` — synchronization byte. Triggers Flash memory preparation for new firmware.
- `0xDD` — erases old firmware without writing new one.

**Bootloader Responses:**
- `0xBB` — successful preparation/erasure. The bootloader is ready to receive firmware packets.
- `0xCC` — memory erasure error. You can resend the sync byte (`0xAA`).

> **Note:** Responses to `0xDD` are the same as for `0xAA`, but after erasure the bootloader does not expect firmware packets.

### Step 2. Firmware Transmission

After receiving `0xBB`, send firmware packets. Packet format:

- `0x01/0xFF/DATA_LEN_H/DATA_LEN_L/DATA`


- `0x01` — header (data for bootloader).
- `0xFF` — code (firmware transmission).
- `DATA_LEN_H` and `DATA_LEN_L` — two bytes indicating data length.
- `DATA` — firmware chunk (1024 bytes).

### Step 3. CRC Checksum Transmission

The final packet contains the CRC checksum calculated for the firmware file:

- `0x01/0xEE/CRC_H/CRC_L`


- `0x01` — header (data for bootloader).
- `0xEE` — code (checksum transmission).
- `CRC_H` and `CRC_L` — two bytes of the checksum.

**Bootloader Responses after CRC Check:**
- `0xE2` — successful update. The bootloader closes interfaces and jumps to the main program.
- `0xE1` — update error. The bootloader returns to sync byte waiting.
- `0xE0` — unknown error.

## Programming the Microcontroller

The bootloader is written to the microcontroller:
- using **STM32CubeIDE** and **ST‑LINK V2** programmer;
- or via **STM32CubeProgrammer** and a programmer.

**Write Addresses:**
- Bootloader: `0x08000000`
- Main Program: `0x08008000`

**Important:**
- The linker script must specify the correct address.
- The interrupt vector must be relocated in the main program.
