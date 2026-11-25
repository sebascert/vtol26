# Visual meshes

Flightory Stallion print meshes use millimetres and part-specific origins.
`model.sdf` assembles them with visual poses, then rotates their span/up/aft
axes into the body forward/left/up frame. The raw STL files remain unchanged.
These reconstructed visual placements are not measured CG or actuator geometry;
physics continues to come from `config/vehicle.json`.

The added `tail/` and `wing/` files are renamed, byte-identical V2 STL files
from the supplied `STALLION-FILES-20260409T012023Z-3-001.zip`, under
`STALLION-FILES/STALLION FILES/V2/TAIL/` and `WING/`. Flightory retains the
original design rights. The tail boom is a visual 16 mm × 435 mm cylinder,
following the supplied assembly manual. Control surfaces remain fixed visuals.
