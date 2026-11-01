# RiverExtract: PLC program (Delta DVP-14SS2, WPLSoft)

This program replaces the old one. It adds:

- an industrial intake sensor, with a wave filter;
- the three bin-full signals from the Raspberry Pi's relays: one bin full slows the belt, two or more full stop it;
- belt speed control (PWM);
- automatic jam reverse with retry limits and a latched fault.

Enter it in WPLSoft's instruction mode and download it to the PLC. Once the PLC runs it, what the paper says about the PLC is what the hardware does.

Assumptions:

- the transistor-output model **DVP-14SS211T** (PWM needs transistor outputs);
- inputs wired so that **+24 V on a terminal turns it on** (S/S to 0 V).

---

## 1. I/O map

The addresses match the existing wiring in the team report (Table 4): X0 intake (the Omron replaces the limit switch on the same terminal), X2 jam, X3 E-stop, X4 pause, Y0/Y1 belt. The only new inputs are the three bin relays on X5–X7.


| Address | Device | Signal |
|---|---|---|
| X0 | Intake debris sensor, retro-reflective, IP67, PNP (see section 5) | ON = beam blocked by floating debris |
| X1 | spare (old ESP32 bin-level wire, not used) | |
| X2 | Jam sensor (IR beam at the head discharge) | ON = beam blocked |
| X3 | Emergency stop, **normally closed** | ON = healthy, OFF = stop |
| X4 | Pause/resume from the web app (Pi relay K4) | ON = pause |
| X5 | Relay K1 on the Pi: PLASTIC bin full (ultrasonic) | ON = full |
| X6 | Relay K2 on the Pi: ORGANIC bin full | ON = full |
| X7 | Relay K3 on the Pi: OTHER bin full | ON = full |
| Y0 | BTS7960 RPWM (existing wire) | belt forward, PWM speed |
| Y1 | BTS7960 LPWM (existing wire) | belt reverse, full speed |
| Y2, Y3 | bin rail (L298N) | not used by this program; leave wired |
| Y4 | Alarm beacon | blinking = latched jam fault; steady = 2+ bins full |
| Y5 | Run lamp | belt moving |

**Relay wiring.** Each relay's COM goes to +24 V from the battery and its NO contact to its X input. The Pi and the PLC never share a wire, so the only link between them is three or four galvanically isolated, one-way contacts.

**Optional fail-safe version (recommended).** Use the NC contacts instead, so that "relay energized" means "bin has room". A crashed Pi or a broken wire then reads as full and stops the belt. In the program, change `LD X5 / X6 / X7` to `LDI` (and `AND` to `ANI`, `OR` to `ORI`) in network 3.

**E-stop.** It must also cut the motor supply in hardware. The PLC logic below is a second layer, not the safety function itself.

---

## 2. Program (instruction list)

```
; ===== N1  E-stop circuit healthy (NC contact) ==============================
LD   X3
OUT  M0            ; M0 = E-stop healthy

; ===== N2  debris at the intake, wave filter ================================
LD   X0
TMR  T0   K3       ; beam blocked for >= 0.3 s (rejects splashes and ripples)
LD   T0
PLS  M3            ; M3 = one-scan pulse per debris event

; ===== N3  bins full (ultrasonic sensors -> Pi -> relays K1..K3) ============
LD   X5
AND  X6
LD   X5
AND  X7
ORB
LD   X6
AND  X7
ORB
OUT  M10           ; M10 = two or more bins full  -> stop
LD   X5
OR   X6
OR   X7
ANI  M10
OUT  M11           ; M11 = exactly one bin full   -> slow

; ===== N4  run latch, re-triggered by every debris event ====================
LD   M3
AND  M0
ANI  M10
ANI  M30
SET  M1            ; M1 = conveyor commanded to run
LD   M1
ANI  M3
TMR  T1   K600     ; run on for 60 s after the last debris event
LD   T1
OR   M10
ORI  M0
OR   M30
RST  M1

; ===== N5  forward drive request ============================================
LD   M1
AND  M0
ANI  X4            ; paused from the web app
ANI  M10
ANI  M20           ; jam recovery running
ANI  M30           ; latched fault
OUT  M5            ; M5 = belt forward

; ===== N6  jam detection and automatic reverse ==============================
LD   M5
AND  X2
TMR  T2   K30      ; jam beam blocked for 3 s while running forward
LD   T2
SET  M20           ; start recovery
LD   M20
TMR  T3   K20      ; reverse for 2 s
LD   M20
ANI  T3
AND  M0
OUT  M21           ; M21 = belt reverse
LD   T3
TMR  T4   K5       ; then stop 0.5 s
LD   T4
CNT  C0   K3       ; count recovery attempts
LD   T4
RST  M20           ; recovery done, forward resumes
LD   C0
SET  M30           ; 3 attempts without clearing -> latched fault
LD   M5
ANI  X2
TMR  T5   K100     ; 10 s of clean running ...
LD   T5
RST  C0            ; ... resets the attempt counter
LDI  M0
RST  M30           ; fault acknowledged by pressing and releasing the E-stop
LDI  M0
RST  C0
LDI  M0
RST  M20

; ===== N7  belt speed: PWM on Y0 ============================================
LD   M1002
MOV  K10  D11      ; PWM period 10 ms (100 Hz)
LD   M1000
MOV  K9   D10      ; normal: 90 % duty
LD   M11
MOV  K5   D10      ; one bin full: 50 % duty
LD   M5
PWM  D10  D11  Y0  ; belt forward

; ===== N8  belt reverse =====================================================
LD   M21
ANI  M5
OUT  Y1            ; belt reverse, full speed (jam recovery only)

; ===== N9  lamps ============================================================
LD   M30
AND  M1013         ; 1-s clock -> blinking for a latched fault
LD   M10
ORB                ; steady for two or more bins full
OUT  Y4
LD   M5
OR   M21
OUT  Y5            ; run lamp

END
```

Timers T0–T5 count in 100-ms units (K30 = 3.0 s). M1000 is always on in RUN, M1002 is on for the first scan, and M1013 is the 1-s clock.

PWM (API 58) uses 1-ms units here. Check API 58 in WPLSoft's help for your unit: a finer time base gives a quieter, higher-frequency PWM. Set the two duty values (K9 and K5) so the belt runs at the speeds you want, then measure those speeds for the paper.

---

## 3. What the program does

1. **Start.** Floating debris that keeps the intake beam blocked for at least 0.3 s starts the belt. Every new debris event restarts a 60-s run-on timer, so the belt stops one minute after the last item.
2. **Bins.**
   - When one bin is full, the belt drops to the reduced duty. The Pi meanwhile sends that bin's class to another bin, so collection continues.
   - When two or more bins are full, the belt stops and the beacon lights steadily.
3. **Jam.**
   - If the jam beam stays blocked for 3 s while the belt runs forward, the belt reverses for 2 s, stops for 0.5 s, and resumes.
   - After three attempts without 10 s of clean running in between, the belt stops with a blinking beacon until the E-stop is pressed and released.
4. **Pause.** The web app can pause the belt through relay K4. The vision system cannot start the belt and has no access to the E-stop chain.

---

## 4. Draft paper text (machine layer)

> A Delta DVP-14SS2 PLC runs the belt independently of the vision layer. Floating debris that blocks a retro-reflective intake sensor for 0.3 s starts the belt; each new detection restarts a 60-s run-on timer. A Gimson GR-WM4-V3 24-V worm-gear motor (60:1) drives the belt through a BTS7960 H-bridge, with speed set by PWM. A jam beam blocked for 3 s triggers a 2-s reverse; three failed attempts latch a fault that is cleared by the emergency stop. An ultrasonic sensor above each bin reports its fill level to the Raspberry Pi, which closes one relay per full bin. The relay contacts switch 24 V into three PLC inputs, the only link between the two controllers. One full bin slows the belt while the Pi re-routes that bin's class; two full bins stop it.

---

## 5. Intake sensor (replaces the lever limit switch)

**Recommended:** Omron E3ZM-B81 (PNP) retro-reflective photoelectric sensor with the E39-RP1 reflector, mounted across the intake mouth.

- It is designed to detect transparent PET bottles: the reference object is a 500-ml bottle 65 mm in diameter. Floating bottles are mostly clear, so an ordinary diffuse sensor misses many of them.
- Stainless housing rated IP67 / IP69K, 10–30 V DC, 1-ms response, wires straight to X0.
- Range is 100–500 mm with the reflector. For the 0.6-m full-scale intake, use the longer-range E3Z-B model.

**Mounting:**

- Sensor on one side of the intake, reflector on the other.
- Beam horizontal, 2–3 cm above the still-water line, so a floating bottle crosses it but small ripples don't.
- The 0.3-s filter in network N2 rejects splashes. If waves still trigger it, raise the beam or lengthen T0.
