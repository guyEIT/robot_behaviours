# pylabrobot — iSWAP off-deck handoff, questions

We're trying to drive a plate from an on-deck carrier (rails=20) to an
off-deck incubator dock at X ≈ -219 mm on a Hamilton STAR via the atomic
path (`lh.move_resource(..., use_unsafe_hotel=True)`). The 6-stage manual
jog recipe works; the atomic one doesn't, and we suspect we're holding
something the wrong way round. Four questions where your guidance would
help.

---

## 1. Is there a way to address the iSWAP shoulder and wrist independently in the atomics?

Our teach-pendant calibration for the incubator dock is
`shoulder=LEFT, wrist=STRAIGHT`. We can reproduce it through the manual
helpers (`rotate_iswap_rotation_drive` + `rotate_iswap_wrist`) but we
can't find a way to express it through `GripDirection`:

- `GripDirection.LEFT` — plate ends up rotated 180° in the gripper
  (looks like shoulder LEFT but the wrist is offset from STRAIGHT).
- `GripDirection.RIGHT` — plate orientation correct, but the arm now
  extends the wrong way and can't reach a left-side dock.

None of the four `GripDirection` values produce the
`shoulder=LEFT, wrist=STRAIGHT` combo we need.

Is there an existing way to pass the two drives separately to the
atomics (`iswap_get_plate`, `unsafe.put_in_hotel`), or is the manual-jog
path the expected route for calibrations like this?

---

## 2. Is `use_unsafe_hotel` meant to be set per-side?

For an on-deck → off-deck transfer, only the drop side should run the
hotel firmware command — calling `get_from_hotel` (PO) on the on-deck
plate hits `R027 PositionNotReachable`. `LiquidHandler.move_resource`
forwards `**backend_kwargs` to both `pick_up_resource` and
`drop_resource`, so `use_unsafe_hotel=True` goes to both.

Is there a supported way to split kwargs per side that we've missed?
We've worked around it by calling `pick_up_resource` and `drop_resource`
separately, but it felt like going around the one-call `move_resource`
API.

---

## 3. What happens if `pickup_direction` differs from `drop_direction` on the atomic path?

If we pick up with `GripDirection.FRONT` and then
`put_in_hotel(grip_direction=LEFT)`, the firmware atomic fails with
`R027`. The PO/PI commands don't seem to rotate the wrist to match
`gr` during the approach; they plan as if the arm is already oriented.

Is there a layer we should be going through that inserts a wrist
rotation between pickup and drop when the directions differ, or is
matching them a hard caller requirement?

---

## 4. Post-init pose + first rotation crashes the wrist

After `initialize_iswap` (FI) the arm sits at Y ≈ 626 mm. Any atomic
that needs a shoulder rotation from that pose binds the wrist against
the back wall:

```
R002/82: Wrist twist drive movement error: drive locked or incremental sensor fault
```

`park_iswap` (PG) doesn't help — it also moves Y far back. We pre-park
manually with `move_iswap_y(330)` before any iSWAP atomic, which works,
but it feels like we're papering over something the atomic should
handle itself. Is manual pre-parking the intended pattern, or is there
a helper that does the right thing?

---

Happy to send diffs, teach files, or firmware traces for any of the
above if useful. The manual-jog path is fine as a fallback, but we'd
like to get `move_resource` working for handoffs where we can.
