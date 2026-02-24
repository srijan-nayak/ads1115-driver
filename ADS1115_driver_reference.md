# ADS1115 C++ Driver Reference
> Compressed from TI SBAS444E (Dec 2024). Targets ADS1115 only. All values verified from datasheet.

---

## 1. Device Overview

| Parameter | Value |
|-----------|-------|
| Resolution | 16-bit, 2's complement |
| Interface | I²C (Standard/Fast/High-Speed: 100kHz / 400kHz / 3.4MHz) |
| Supply | 2.0V – 5.5V |
| Input channels | 4 single-ended **or** 2 differential (via MUX) |
| Sample rates | 8, 16, 32, 64, 128, 250, 475, 860 SPS |
| PGA ranges | ±6.144V, ±4.096V, ±2.048V, ±1.024V, ±0.512V, ±0.256V |
| Current (active) | 150 µA typ, 300 µA max |
| Current (power-down) | 0.5 µA typ |
| Temp range | –40°C to +125°C |
| Conversion time | 1 / DR seconds (single-cycle settling) |
| INL | 1 LSB max (best-fit, DR=8SPS, FSR=±2.048V) |

**Key features vs siblings:**
- ADS1113: No PGA, no comparator, 1 differential input (FSR fixed ±2.048V)
- ADS1114: PGA + comparator, 1 differential input
- **ADS1115: PGA + comparator + 4-ch MUX** ← this document

---

## 2. I²C Addressing

| ADDR pin connection | 7-bit address | 8-bit write | 8-bit read |
|---------------------|--------------|-------------|------------|
| GND | 0x48 | 0x90 | 0x91 |
| VDD | 0x49 | 0x92 | 0x93 |
| SDA | 0x4A | 0x94 | 0x95 |
| SCL | 0x4B | 0x96 | 0x97 |

> ADDR is sampled **continuously**. Prefer GND/VDD/SCL before SDA. If using SDA as address, hold SDA low ≥100ns after SCL goes low during I²C communication.

Up to **4 devices** on one bus (one per address option).

---

## 3. Register Map

| P[1:0] | Register | Size | Reset |
|--------|----------|------|-------|
| 0b00 | Conversion | 16-bit, R | 0x0000 |
| 0b01 | Config | 16-bit, R/W | 0x8583 |
| 0b10 | Lo_thresh | 16-bit, R/W | 0x8000 |
| 0b11 | Hi_thresh | 16-bit, R/W | 0x7FFF |

### Address Pointer Register (write-only, sent as 2nd byte after device address)
```
Bits [7:2] = 0b000000 (reserved, always 0)
Bits [1:0] = P[1:0]  register select
```

---

## 4. Config Register (0x8583 default)

**Bit layout (MSB first):**

```
Bit  15   14:12    11:9    8      7:5    4          3         2        1:0
     OS   MUX[2:0] PGA[2:0] MODE  DR[2:0] COMP_MODE COMP_POL COMP_LAT COMP_QUE[1:0]
```

### Bit 15 — OS (Operational Status)
| Value | Write meaning | Read meaning |
|-------|--------------|--------------|
| 0 | No effect | Conversion in progress |
| 1 | Start single conversion (power-down state only) | Device idle |

### Bits 14:12 — MUX[2:0] (Input Multiplexer)
| MUX[2:0] | AINP | AINN | Use case |
|----------|------|------|----------|
| 0b000 | AIN0 | AIN1 | Differential (default) |
| 0b001 | AIN0 | AIN3 | Differential |
| 0b010 | AIN1 | AIN3 | Differential |
| 0b011 | AIN2 | AIN3 | Differential |
| 0b100 | AIN0 | GND | Single-ended |
| 0b101 | AIN1 | GND | Single-ended |
| 0b110 | AIN2 | GND | Single-ended |
| 0b111 | AIN3 | GND | Single-ended |

### Bits 11:9 — PGA[2:0] (Programmable Gain Amplifier)
| PGA[2:0] | FSR | LSB size | Notes |
|----------|-----|----------|-------|
| 0b000 | ±6.144V | 187.5 µV | ⚠️ Do not exceed VDD+0.3V on inputs |
| 0b001 | ±4.096V | 125.0 µV | ⚠️ Same warning |
| 0b010 | ±2.048V | 62.5 µV | **Default** |
| 0b011 | ±1.024V | 31.25 µV | |
| 0b100 | ±0.512V | 15.625 µV | |
| 0b101/110/111 | ±0.256V | 7.8125 µV | |

> **LSB formula:** `LSB = FSR / 2^16` (FSR = full positive range, e.g. 2.048 for ±2.048V gives 62.5µV)
> **Voltage formula:** `V = code × LSB` where code is signed int16_t

### Bit 8 — MODE
| Value | Mode |
|-------|------|
| 0 | Continuous-conversion |
| 1 | Single-shot / power-down (default) |

### Bits 7:5 — DR[2:0] (Data Rate)
| DR[2:0] | SPS | Conv. time |
|---------|-----|------------|
| 0b000 | 8 | 125 ms |
| 0b001 | 16 | 62.5 ms |
| 0b010 | 32 | 31.25 ms |
| 0b011 | 64 | 15.6 ms |
| 0b100 | 128 | 7.8 ms (default) |
| 0b101 | 250 | 4.0 ms |
| 0b110 | 475 | 2.1 ms |
| 0b111 | 860 | 1.16 ms |

> Data rate has ±10% tolerance. Scales with internal oscillator (1 MHz, drifts with temp).

### Bit 4 — COMP_MODE
| Value | Mode |
|-------|------|
| 0 | Traditional comparator (hysteresis: asserts above Hi_thresh, deasserts below Lo_thresh) |
| 1 | Window comparator (asserts outside Lo_thresh–Hi_thresh window) |

### Bit 3 — COMP_POL
| Value | ALERT/RDY polarity |
|-------|-------------------|
| 0 | Active low (default) |
| 1 | Active high |

### Bit 2 — COMP_LAT
| Value | Behavior |
|-------|---------|
| 0 | Non-latching: ALERT/RDY deasserts automatically (default) |
| 1 | Latching: stays asserted until Conversion register is read or SMBus alert response |

### Bits 1:0 — COMP_QUE[1:0]
| Value | Behavior |
|-------|---------|
| 0b00 | Assert after 1 conversion beyond threshold |
| 0b01 | Assert after 2 conversions beyond threshold |
| 0b10 | Assert after 4 conversions beyond threshold |
| 0b11 | **Disable comparator, ALERT/RDY = high-Z (default)** |

---

## 5. Conversion Register

- 16-bit signed 2's complement
- Read MSB first, then LSB
- Reset to 0x0000 on power-up; stays 0x0000 until first conversion completes
- Clips at 0x7FFF (+FS) and 0x8000 (–FS)

```
Code 0x7FFF = +FS (32767)
Code 0x0001 = +FS/32767
Code 0x0000 = 0V
Code 0xFFFF = –1 LSB
Code 0x8000 = –FS (–32768)
```

**Single-ended note:** Only uses 0x0000–0x7FFF range (0V to +FS). Negative codes possible near 0V due to offset.

---

## 6. Threshold Registers (Lo_thresh / Hi_thresh)

- 16-bit signed 2's complement, same format as Conversion register
- Must be updated when PGA changes (digital comparator uses same scale)
- Hi_thresh must always > Lo_thresh for comparator function

**Conversion-Ready pin mode** (special use of threshold registers):
```
Hi_thresh MSB = 1  →  Hi_thresh = 0x8000
Lo_thresh MSB = 0  →  Lo_thresh = 0x7FFF (or any value with MSB=0)
COMP_QUE != 0b11   →  keep ALERT/RDY enabled
```
In this mode: ALERT/RDY pulses ~8µs at end of each conversion (continuous mode), or asserts low at end of conversion (single-shot, COMP_POL=0).

---

## 7. I²C Communication Protocol

### Write to Register
```
START
[ADDR_BYTE: 7-bit addr | W(0)]  → ACK
[POINTER: 0b000000 | P[1:0]]    → ACK
[DATA_MSB]                       → ACK
[DATA_LSB]                       → ACK
STOP
```

### Read from Register
```
# Step 1: Set pointer (optional if pointer already set)
START
[ADDR_BYTE: 7-bit addr | W(0)]  → ACK
[POINTER: 0b000000 | P[1:0]]    → ACK
STOP

# Step 2: Read
START
[ADDR_BYTE: 7-bit addr | R(1)]  → ACK
← [DATA_MSB]                    ACK by controller
← [DATA_LSB]                    NACK by controller (or ACK to continue)
STOP
```

> Pointer register retains last written value — no need to resend for repeated reads of same register.

### Quick-Start: Continuous Mode + Read
```
Write Config (0x90, 0x01, 0xC4, 0x83):
  0x90 = device write addr (ADDR=GND)
  0x01 = point to Config register
  0xC4 = 1100 0100 → OS=1, MUX=100(AIN0/GND), PGA=010(±2.048V), MODE=0(continuous)
  0x83 = 1000 0011 → DR=000(8SPS)... adjust as needed

Write Pointer to Conversion (0x90, 0x00)
Read 2 bytes (0x91) → combine as int16_t
```

### SMBus Alert Response (latching comparator clear)
```
START
[0x0C | R(1)]  (SMBus alert addr = 0b0001100x)  → ACK by asserting device
← [device I²C addr | status_bit]                 NACK by controller
STOP
```
Lowest-address device wins arbitration. Repeat until all devices cleared.

### General Call Reset
```
START
[0x00]  → ACK
[0x06]  → ACK   (reset command)
STOP
```
Equivalent to power-on reset. All registers return to defaults.

---

## 8. Operating Modes

### Single-Shot Mode (MODE=1, default)
1. Device is in power-down state (still responds to I²C)
2. Write OS=1 to Config register → device wakes (~25µs), converts, powers down
3. Poll OS bit: 0 = converting, 1 = done
4. Read Conversion register

### Continuous-Conversion Mode (MODE=0)
1. Write MODE=0 to Config register
2. Device converts continuously at programmed DR
3. Read Conversion register at any time (always returns latest completed conversion)
4. New config takes effect after current conversion completes

### Duty-Cycling for Low Power
- Set MODE=1, DR=860SPS
- MCU requests single-shot every N ms
- At 860SPS, conversion takes ~1.2ms → device sleeps remainder
- Example: trigger every 125ms → ~1/100th power of continuous at same effective rate

---

## 9. ALERT/RDY Pin

- **Open-drain output** — requires external pullup to VDD
- Default state: high-impedance (COMP_QUE=0b11)
- Functions: comparator alert OR conversion-ready signal

| Configuration | Behavior |
|---------------|---------|
| COMP_QUE = 11 | Pin disabled (high-Z) |
| Comparator mode | Asserts per COMP_MODE/COMP_POL/COMP_LAT/COMP_QUE settings |
| Conv-ready mode | ~8µs pulse per conversion (continuous) or holds asserted (single-shot) |

---

## 10. Electrical Specs for Driver Design

| Parameter | Value |
|-----------|-------|
| VIH (digital input) | 0.7×VDD min |
| VIL (digital input) | 0.3×VDD max |
| VOL (open-drain output) | 0.4V max @ IOL=3mA |
| Input leakage | ±10 µA max |
| Analog input range | GND to VDD (absolute); FSR sets scale |
| Analog input protection | ESD diodes to rails; keep inputs within GND–0.3V to VDD+0.3V |
| Max continuous input current | 10mA |
| I²C timeout | 25ms bus idle |
| Power-up to ready | 50µs after VDD stable |
| I²C Fast mode | SCL up to 400kHz, pullups 1k–10kΩ typical |
| I²C High-speed mode | SCL up to 3.4MHz; requires Hs controller code preamble |

### Noise (RMS, VDD=3.3V, inputs shorted)
| DR | FSR ±2.048V | FSR ±0.256V |
|----|------------|------------|
| 8 SPS | 62.5 µV | 7.81 µV |
| 128 SPS | 62.5 µV | 7.81 µV |
| 860 SPS | 62.5 µV | 7.81 µV |

> RMS noise is constant across data rates for this device (oversampling already factored in). Peak-to-peak noise increases at higher rates.

---

## 11. C++ Driver Architecture Notes

### Recommended Enums/Constants
```cpp
// I²C Addresses
enum class ADS1115Address : uint8_t {
    GND = 0x48, VDD = 0x49, SDA = 0x4A, SCL = 0x4B
};

// Register pointers
enum class ADS1115Reg : uint8_t {
    CONVERSION = 0x00, CONFIG = 0x01, LO_THRESH = 0x02, HI_THRESH = 0x03
};

// MUX channel
enum class ADS1115Mux : uint16_t {
    DIFF_0_1 = 0x0000, DIFF_0_3 = 0x1000, DIFF_1_3 = 0x2000, DIFF_2_3 = 0x3000,
    SINGLE_0 = 0x4000, SINGLE_1 = 0x5000, SINGLE_2 = 0x6000, SINGLE_3 = 0x7000
};

// PGA / FSR
enum class ADS1115PGA : uint16_t {
    FSR_6144 = 0x0000, FSR_4096 = 0x0200, FSR_2048 = 0x0400,  // default
    FSR_1024 = 0x0600, FSR_0512 = 0x0800, FSR_0256 = 0x0A00
};

// Data rate
enum class ADS1115DR : uint16_t {
    SPS_8=0x0000, SPS_16=0x0020, SPS_32=0x0040, SPS_64=0x0060,
    SPS_128=0x0080, SPS_250=0x00A0, SPS_475=0x00C0, SPS_860=0x00E0
};

// Comparator mode
enum class ADS1115CompMode : uint16_t { TRADITIONAL=0x0000, WINDOW=0x0010 };
enum class ADS1115CompPol  : uint16_t { ACTIVE_LOW=0x0000, ACTIVE_HIGH=0x0008 };
enum class ADS1115CompLat  : uint16_t { NON_LATCHING=0x0000, LATCHING=0x0004 };
enum class ADS1115CompQue  : uint16_t { AFTER_1=0x0000, AFTER_2=0x0001, AFTER_4=0x0002, DISABLED=0x0003 };

// Config register bit masks
constexpr uint16_t ADS1115_OS_START   = 0x8000; // write: start single conversion
constexpr uint16_t ADS1115_OS_BUSY    = 0x8000; // read: 0=converting, 1=idle
constexpr uint16_t ADS1115_MODE_CONT  = 0x0000;
constexpr uint16_t ADS1115_MODE_SINGLE= 0x0100;

// Default config: single-shot, AIN0/GND, ±2.048V, 128SPS, comp disabled
constexpr uint16_t ADS1115_CONFIG_DEFAULT = 0x8583;
```

### LSB Voltage per PGA Setting
```cpp
constexpr float ADS1115_LSB_uV[] = {
    187.5f,  // FSR ±6.144V  (PGA=000)
    125.0f,  // FSR ±4.096V  (PGA=001)
    62.5f,   // FSR ±2.048V  (PGA=010) default
    31.25f,  // FSR ±1.024V  (PGA=011)
    15.625f, // FSR ±0.512V  (PGA=100)
    7.8125f, // FSR ±0.256V  (PGA=101/110/111)
};

// Voltage (V) = (int16_t)raw_code * lsb_uV * 1e-6
```

### Conversion-Ready via ALERT/RDY
```cpp
// To enable conversion-ready mode:
writeRegister(ADS1115Reg::HI_THRESH, 0x8000); // MSB=1
writeRegister(ADS1115Reg::LO_THRESH, 0x0000); // MSB=0
// Set COMP_QUE to anything except 0b11 in Config
// Then ALERT/RDY pulses ~8µs after each conversion
```

### Single-Shot Read Sequence
```cpp
// 1. Write config with OS=1 and desired MUX/PGA/DR
uint16_t config = ADS1115_OS_START | mux | pga | ADS1115_MODE_SINGLE | dr | ADS1115_COMP_DISABLED;
writeRegister(ADS1115Reg::CONFIG, config);

// 2a. Poll method: wait conv_time_ms, then check OS bit
//     OR poll until OS bit = 1 (device idle)
// 2b. Interrupt method: wait for ALERT/RDY assert (if conv-ready mode enabled)

// 3. Read conversion register
int16_t raw = (int16_t)readRegister(ADS1115Reg::CONVERSION);

// 4. Convert to voltage
float voltage = raw * lsb_uV * 1e-6f;
```

### Driver Method Checklist
- [ ] `begin(address)` — init I²C, verify comms, wait 50µs post-power
- [ ] `setMux(channel)` — set MUX[2:0]
- [ ] `setPGA(fsr)` — set PGA[2:0], also update threshold regs if comparator in use
- [ ] `setDataRate(dr)` — set DR[2:0]
- [ ] `setMode(continuous|single)` — set MODE bit
- [ ] `startConversion()` — write OS=1 (single-shot mode)
- [ ] `isConversionReady()` — poll OS bit in Config
- [ ] `readRaw()` → `int16_t` — read Conversion register
- [ ] `readVoltage()` → `float` — readRaw × LSB
- [ ] `readChannel(ch)` — combined: set MUX, trigger, wait, read (single-shot)
- [ ] `setComparator(mode, pol, lat, que, lo, hi)` — configure comparator
- [ ] `enableConvReadyPin(bool)` — configure ALERT/RDY as conv-ready
- [ ] `setThresholds(lo, hi)` — write Lo/Hi_thresh registers
- [ ] `reset()` — I²C general call reset (0x00, 0x06)
- [ ] `powerDown()` — MODE=1 without OS=1 (already in power-down after single-shot)

---

## 12. Hardware / PCB Notes (for driver docs)

- **Pullup resistors**: 1kΩ–10kΩ on SDA and SCL (lower for faster speeds / longer traces)
- **Decoupling cap**: 0.1µF ceramic (MLCC, low ESR) as close to VDD pin as possible
- **ALERT/RDY**: connect pullup to VDD when used; leave floating or tie to VDD when unused
- **Unused analog inputs**: float or tie to GND
- **Absolute max analog input**: GND–0.3V to VDD+0.3V; use series resistor + Schottky clamp if overvoltage possible
- **Overdriving one input** on ADS1115 can corrupt conversions on other channels — clamp signals externally
- **Differential capacitor**: place C0G/NP0 ceramic diff cap across AINP/AINN at ADC inputs for noise filtering
- **RC anti-alias filter**: f_cutoff ≈ DR or 10×DR; filter resistors <1kΩ to avoid gain error from input bias current interaction

---

## 13. Timing Summary

| Parameter | Fast Mode | High-Speed Mode |
|-----------|-----------|----------------|
| Max SCL | 400 kHz | 3.4 MHz |
| Bus free time (tBUF) | 600 ns | 160 ns |
| Hold time after START (tHD_STA) | 600 ns | 160 ns |
| Data setup (tSU_DAT) | 100 ns | 10 ns |
| SCL low period | 1300 ns | 160 ns |
| SCL high period | 600 ns | 60 ns |
| Rise/fall time (SDA, SCL) | 300 ns | 160 ns |

> High-speed mode: send Hs controller code `0b00001xxx` after START (not acknowledged by ADS1115). Device exits Hs mode on next STOP.
