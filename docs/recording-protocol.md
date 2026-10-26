# Recording protocol

Goal: a **sound target** for one real engine, to judge the simulation against
(`VISION.md`, success criterion). It was first written for a Yamaha MT-07 with
an Arrow exhaust, and works for any engine. Without a
known engine speed, a recording only serves the ear; with a known engine speed,
it serves to measure the simulation/reality gap band by band. Everything below
aims at that second use.

## Before starting: write down

- Engine and vehicle: make, model, year, engine code.
- Exact exhaust: stock or aftermarket, make and part number, slip-on or full
  system, catalyst kept or not, **dB-killer fitted or removed**. The timbre
  depends heavily on it.
- Mileage, stock air filter or not, modified ECU map or
  not.
- Approximate outside temperature, wind.

## Equipment

- **Ideal:** a portable recorder (Zoom H1n/H4n type, €80–150), WAV
  48 kHz / 24 bit, manual gain.
- **Acceptable:** a phone, **provided every processing stage is turned off**:
  automatic gain control, noise reduction, "concert mode". Use a recording app
  that allows a fixed gain and WAV output. Automatic processing destroys
  precisely the timbre we want to measure.
- A **second phone filming the dashboard** for the whole take, to read the
  engine speed. A clap in front of both at the start of each take lets them be
  synchronised.

## Setting the level

Do a high-rpm trial before the real take and check that **no peak clips**
(nothing at the ceiling of the waveform). The chosen level never changes
afterwards, otherwise the takes are no longer comparable with each other.

## Microphone positions

Always the same, measured with a tape measure, microphone at the height of the
exhaust outlet (one per outlet side if there are several):

- **A — close:** 50 cm from the outlet, at 45° to the jet axis (never in the
  jet: the blast saturates the microphone).
- **B — scene:** 4 m to the side of the vehicle, on the exhaust side. This is
  the default listening distance of the simulation, so the most directly
  comparable.

Record the whole sequence at A, then the whole sequence at B. Outdoors, in an
open space, away from walls and parked cars (reflections change the timbre),
with no wind.

## Sequence (warm engine, stationary)

The engine speeds below suit a motorcycle twin; scale them to the engine
(stay below about two thirds of its redline while stationary).

**Warm** engine (after at least 10 minutes of riding): gas temperature changes
the speed of sound in the exhaust, and therefore its resonances.

1. Clap, then **idle** for 20 s.
2. **Steady holds**, keeping the throttle as constant as possible:
   2,000, 3,000, 4,000, 5,000 rpm, about 5 s each. Let it drop back to idle for
   a few seconds between two holds.
3. **Slow sweep**: from idle to 6,000 rpm in about 8 s, then release.
4. **Blips**: three sharp blips up to ~6,000 rpm, released firmly. This is the
   take that captures any overrun pops.
5. Final clap.

Go easy on the engine: no long hold above 5,000 rpm while stationary, let the
fan run between series, and be considerate of the neighbours.

## Optional, later

- A riding take (pass-by in front of the microphone with gear and engine speed
  noted): it is the only take **under load**, but the engine speed is harder to
  know.
- The same sequence with the dB-killer swapped (fitted/removed): a change of a
  single factor, ideal for checking that the simulation reacts like reality.

## Delivery

Share the files under CC0 or CC BY 4.0 (a **Reference recording** issue on
GitHub), with a short `notes.md`; in the repository they go in
`references/<engine>-<exhaust>/`. The notes give the
equipment, position (A/B), gain level, conditions, and for each file the time
(in seconds) of each hold and each blip, read from the dashboard video.
