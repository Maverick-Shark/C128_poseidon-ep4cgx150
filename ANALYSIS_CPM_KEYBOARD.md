# C128 MiST Port - CP/M Keyboard Failure Analysis

## 1. Problem Description

The Commodore C128 core ported to MiST platforms (Poseidon EP4CGX150 and Calypso CYC-1000)
has a keyboard that **works correctly in C128 native mode and C64 mode**, but
**fails completely in CP/M mode**.

The original MiSTer core works correctly in all three modes.

## 2. Architecture Overview

### Keyboard Signal Flow

```
PS/2 Keyboard → user_io → ps2_key[10:0] → key processing → fpga64_sid_iec → fpga64_keyboard → CIA1 → CPU
```

- **C128/C64 modes**: The 8502 (T65) CPU reads the keyboard through CIA1 Port A/B
- **CP/M mode**: The Z80 (T80) CPU reads the keyboard through the same CIA1, via I/O-mapped access

The `fpga64_keyboard.vhd` module is **identical** between MiSTer and MiST ports — the
keyboard matrix decoder and PS/2 scancode handling are unchanged.

### Key Files Compared

| File | MiST (Poseidon/Calypso) | MiSTer | Status |
|------|------------------------|--------|--------|
| `fpga64_keyboard.vhd` | Same | Same | Identical |
| `mos6526_8520.v` (CIA) | Same | Same | Identical |
| `mmu8722.vhd` | Same | Same | Identical |
| `fpga64_buslogic.vhd` | Older version | Newer version | **Different** |
| `cpu_z80.vhd` | Older version | Newer version | **Different** |
| `fpga64_sid_iec.vhd` | Older version | Newer version | **Significantly different** |
| `guest_top.sv`/`c128.sv` | MiST adaptation | MiSTer original | **Different** |

## 3. Root Cause Analysis

The MiST ports are based on an **older revision** of the MiSTer RTL code. The MiSTer
codebase has since undergone significant rework of the Z80 CPU integration, bus timing,
and I/O handling. The following differences directly affect CP/M keyboard operation:

### 3.1 CRITICAL: `cs_io` Includes `cpuIO_T80` (I/O clock stretching)

**MiST (Poseidon) — `fpga64_sid_iec.vhd` line 1221:**
```vhdl
cs_io <= cs_vic or cs_sid or cs_mmuL or cs_vdc or cs_cia1 or cs_cia2
         or io7_i or ioe_i or iof_i or cpuIO_T80;
```

**MiSTer — `fpga64_sid_iec.vhd` line 726:**
```vhdl
cs_io <= cs_vic or cs_sid or cs_mmuL or cs_vdc or io7_i or cs_color
         or cs_cia1 or cs_cia2 or ioe_i or iof_i;
```

**Impact**: In the MiST version, when the Z80 performs **any** I/O operation (`cpuIO_T80='1'`),
`cs_io` goes high, triggering the clock stretching mechanism. This stretching gates the
Z80 CPU cycle enables (`cpuCycT80`), preventing the Z80 from completing its I/O access
properly. Since the keyboard is read via CIA1 through Z80 I/O operations in CP/M mode,
this directly blocks keyboard reading.

In MiSTer, `cpuIO_T80` is NOT included in `cs_io`, so Z80 I/O operations proceed without
spurious clock stretching.

### 3.2 CRITICAL: `cpu_z80.vhd` — Different Output Latching and Enable Interface

**MiST version:**
```vhdl
entity cpu_z80 is
   port(
      enable  : in  std_logic_vector(1 downto 0);  -- 2-bit enable
      ...
   );

-- Uses combinational write detection:
process(d1mhz0, WR_n, localIORQ_n)
begin
   if WR_n = '0' then
      localWe <= '1';
   elsif d1mhz0(1) = '1' or localIORQ_n = '0' or reset = '1' then
      localWe <= '0';
   end if;
end process;

-- Outputs muxed between direct and latched:
io   <= io_x   when d1mhz1(1) = '1' else io_l;
we   <= we_x   when d1mhz1(1) = '1' else we_l;
addr <= addr_x when d1mhz1(1) = '1' else addr_l;
do   <= do_x   when d1mhz1(1) = '1' else do_l;
```

**MiSTer version:**
```vhdl
entity cpu_z80 is
   port(
      enable  : in  std_logic;          -- 1-bit enable
      latch   : in  std_logic;          -- Separate latch signal
      ...
   );

-- Uses edge-detected write:
WR_f <= WR_f or (WR_n_l and not WR_n);
if latch = '1' then
   WR_f <= '0';
   we <= WR_f or (WR_n_l and not WR_n);
   io <= not IORQ_n;
   addr <= unsigned(localA);
   do <= unsigned(localDo);
end if;
```

**Impact**: The MiSTer version provides clean, stable, edge-triggered output signals via
a dedicated `latch` signal. The MiST version uses combinational muxing between direct and
latched outputs, which can produce glitches and timing issues, particularly during Z80 I/O
operations where data and address stability are crucial for CIA1 access.

### 3.3 SIGNIFICANT: CIA Enable Timing Depends on CPU Type

**MiST version:**
```vhdl
when CYCLE_CPU4 =>
   enableCia_n <= cpuActT65;        -- Only for 8502
when CYCLE_CPU8 =>
   enableCia_n <= not cpuActT65;    -- Only for Z80
```

**MiSTer version:**
```vhdl
when CYCLE_CPU0 =>
   enableCia_n <= '1';              -- Unconditional
```

**Impact**: In MiSTer, the CIA is clocked at a fixed position (CYCLE_CPU0) regardless of
which CPU is active. In MiST, the CIA phi2 negative edge occurs at CYCLE_CPU4 for 8502
or CYCLE_CPU8 for Z80. Combined with issue 3.1 (clock stretching preventing Z80 from
completing cycles), the CIA may never receive proper clock edges during Z80 I/O.

### 3.4 SIGNIFICANT: Z80 Cycle Timing in `fpga64_sid_iec.vhd`

**MiST version** — Z80 gets cycle enables at 3 phases, using `cs_io`/`cs_io_l` for gating:
```vhdl
cpuCycT80 <= "00";  -- 2-bit vector
when CYCLE_CPU2 | CYCLE_CPUA => cpuCycT80(0) <= t80_cyc_s;  (gated by cs_io)
when CYCLE_CPU6 | CYCLE_CPUE => cpuCycT80(1) <= t80_cyc_s;  (gated by cs_io_l)
when CYCLE_CPU7 | CYCLE_CPUF => latch data
```

**MiSTer version** — Z80 gets cycle enables at 4 phases, using `phi0_cpu` for gating:
```vhdl
cpuCycT80 <= '0';   -- 1-bit
cpuLatT80 <= '0';   -- Separate latch
when CYCLE_CPU2 | CYCLE_CPUA => cpuCycT80 (gated by phi0_cpu)
when CYCLE_CPU4 | CYCLE_CPUC => cpuCycT80 (gated by phi0_cpu)
when CYCLE_CPU6 | CYCLE_CPUE => cpuLatT80 (gated by phi0_cpu)
when CYCLE_CPU7 | CYCLE_CPUF => clear t80_cyc_s (gated by phi0_cpu)
```

**Impact**: The MiST version provides fewer Z80 cycle opportunities and uses `cs_io`-based
gating (which is polluted by `cpuIO_T80` as described in 3.1). The MiSTer version uses
`phi0_cpu` — a clean clock phase signal unaffected by I/O state.

### 3.5 MODERATE: `z80_n` Signal Source for Buslogic

**MiST:**
```vhdl
-- buslogic instantiation
z80_n => mmu_z80_n,          -- MMU register (desired state)

-- keyboard module
alt_crsr => not mmu_z80_n,   -- Uses MMU register
```

**MiSTer:**
```vhdl
-- buslogic instantiation
z80_n => not cpuBusAkT80_n,  -- Actual bus state

-- keyboard module
alt_crsr => not cpuBusAkT80_n,
```

**Impact**: The buslogic in MiST uses the MMU's *desired* state for memory/I/O mapping,
while MiSTer uses the *actual* Z80 bus acknowledge. This could cause momentary mismatches
during CPU transitions, though in steady-state CP/M mode both signals should agree.

### 3.6 MODERATE: CPU Data Bus Routing

**MiST:**
```vhdl
cpuDo_T65  <= cpuDo_T65_o when cpuPacc = '0' else vicDi;
cpuAddr_nd <= cpuAddr_T65 when cpuActT65 = '1' else cpuAddr_T80;
cpuDo_nd   <= cpuDo_T65   when cpuActT65 = '1' else cpuDo_T80;
cpuWe_nd   <= cpuWe_T65   when cpuActT65 = '1' else cpuWe_T80;
cpuAddr <= cpuAddr_nd when dma_active = '0' else dma_addr;
cpuDo   <= cpuDo_nd   when dma_active = '0' else dma_dout;
cpuWe   <= cpuWe_nd   when dma_active = '0' else dma_we;
```

**MiSTer:**
```vhdl
cpuAddr <= dma_addr    when dma_active = '1' else
           cpuAddr_T65 when cpuActT65 = '1' else cpuAddr_T80;
cpuDo   <= dma_dout    when dma_active = '1' else
           cpuDo_T65   when (cpuActT65 = '1' and cpuPacc = '0') else
           cpuDo_T80   when cpuBusAkT80_n = '1' else lastVicDi;
cpuWe   <= dma_we      when dma_active = '1' else
           cpuWe_T65   when cpuActT65 = '1' else cpuWe_T80;
```

**Impact**: The MiSTer version has a fallback (`lastVicDi`) when neither CPU is actively
driving the bus. The MiST version uses intermediate signals that could result in stale data.

### 3.7 MINOR: `cs_enable` Definition

**MiST:**
```vhdl
cs_enable <= cpuBusAk_T80_n or (io_enable and (baLoc or cpuWe));
```

**MiSTer:**
```vhdl
cs_enable <= io_enable and (baLoc or cpuWe or not cpuActT65);
```

**Impact**: Both evaluate to '1' in steady-state CP/M mode, but through different logic
paths. When Z80 is active: MiST has `cpuBusAk_T80_n='1'` → `cs_enable='1'`. MiSTer has
`cpuActT65='0'` → `not cpuActT65='1'` → `cs_enable=io_enable`.

## 4. Conclusion

The CP/M keyboard failure is caused by **multiple interrelated issues** in the Z80 CPU
integration, all stemming from the MiST ports using an older revision of the MiSTer RTL.

The **primary cause** is issue 3.1 (`cs_io` including `cpuIO_T80`): when the Z80 tries to
read the keyboard via CIA1 I/O operations, the spurious clock stretching prevents proper
completion of the Z80 bus cycles. This is compounded by issue 3.2 (the older `cpu_z80.vhd`
with its combinational output logic) and issue 3.3 (CIA enable timing being different for
Z80 vs 8502).

The keyboard works in C128 and C64 modes because the 8502 CPU accesses CIA1 through
memory-mapped I/O (not Z80-style port I/O), so `cpuIO_T80` is never asserted, and `cs_io`
functions correctly for clock stretching.

## 5. Proposed Solution

### Approach: Update Z80-Related RTL from MiSTer

The recommended approach is to update the three affected RTL files to match the current
MiSTer versions, focusing on Z80-related sections:

#### Files Requiring Changes:

1. **`rtl/fpga64_sid_iec.vhd`** — Main integration file (most changes)
   - Remove `cpuIO_T80` from `cs_io`
   - Update CIA enable timing (unconditional at CYCLE_CPU0)
   - Update Z80 cycle timing to use `phi0_cpu` gating
   - Update `cpuCycT80` to 1-bit + separate `cpuLatT80`
   - Update `cpuActT65` to use `cpuBusAkT80_n`
   - Update `cs_enable` definition
   - Update CPU data bus routing
   - Update `z80_n` output and buslogic connection
   - Remove `pulseWr` (keep only `pulseWr_io`)
   - Add `dma_pending` logic
   - Update `enableMmu` to use cycle-based logic
   - Update VDC access timing
   - Adapt color RAM write enable

2. **`rtl/cpu_z80.vhd`** — Z80 CPU wrapper
   - Change `enable` from 2-bit to 1-bit
   - Add `latch` input signal
   - Replace combinational `localWe` with edge-detected `WR_f`
   - Replace output muxing with latch-based output capture

3. **`rtl/fpga64_buslogic.vhd`** — Bus logic / PLA
   - Replace `vicHasBus` port with `aec` + `dma_active`
   - Add DMA-aware address translation
   - Update `dma_active` conditions for I/O access gating

#### Risk Assessment:

- **High confidence**: The MiSTer code is proven to work in all three modes
- **Medium risk**: Changes to bus timing could affect C128/C64 mode stability
  if not applied consistently — all three files should be updated together
- **Mitigation**: The identical keyboard, CIA, and MMU modules reduce the
  integration risk. Testing should verify all three modes after the update.

### Alternative Approach: Minimal Fix (Higher Risk)

A minimal fix targeting only `cs_io` (removing `cpuIO_T80`) might partially restore
keyboard function, but is not recommended because the older `cpu_z80.vhd` output
handling and CIA timing issues could still cause unreliable behavior.
