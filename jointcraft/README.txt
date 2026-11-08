JointCraft 3D v0.3.0 — prototype, tested with Blender 5.2.0 LTS

Install ZIP: Edit > Preferences > Get Extensions > menu > Install from Disk.
Disable older separately installed JointCraft versions before installing.

Use:
Select Joint Type: Fixed Pin + Socket or Articulated Hinge.
The following numbered instructions describe Fixed Pin + Socket.
1. Create two closed mesh boxes (e.g. 20 x 20 x 20 mm) sharing a face.
2. Object Mode > N panel > JointCraft.
3. Assign the pin object and socket object.
4. Enable Show live preview to see blue pin and orange socket x-ray guides.
5. Disable Automatic dimensions to change Pin diameter / Pin length (mm).
   Guides follow parameter and object changes without modifying source meshes.
   Oversized joints are rejected with the maximum supported dimensions.
6. Click Create Pin + Socket to commit. Preview switches off on success.
7. Inspect mesh and test fit before printing.

The algorithm refreshes the view layer and checks evaluated WORLD-AXIS-ALIGNED
bounding-box face contact. Objects must belong to the active scene and view layer.
Apply modifiers and resolve shape keys before using this version: the operation
replaces mesh data and does not support preserving a modifier/shape-key stack.
It is intended for boxes / flat-faced aligned parts, not arbitrary sculpted surfaces.
It calculates one fixed cylindrical pin and socket at the overlapping face center.
It does not yet support multiple joints, print-in-place hinges, face normals,
non-axis-aligned parts, or full printability validation.

Default clearance is RADIAL 0.20 mm (socket diameter exceeds pin diameter by 0.40 mm).
Wall setting 1.2 mm is used for conservative bounding-box dimension estimates,
NOT a guarantee of minimum wall thickness in arbitrary meshes.

Both Boolean meshes are prepared before either source mesh is replaced.
On a Boolean error the source mesh data is retained and temporary data is removed.
Successful operations modify both meshes. Ctrl+Z to undo.

0.2.1 fixes:
- Refresh stale transforms before detecting contact; include object names, world
  bounds and tolerance in contact failure messages (BU and mm).
- Filter object pickers to scene meshes and reject off-scene/view-layer objects.
- Correct cutter winding for -X, +Y and -Z.
- Preserve both original meshes if the second Boolean fails.

0.2.2 adds custom diameter and length, plus live GPU wireframe x-ray guides.
Blue includes the embedded pin base; orange includes socket clearance.
Guides are not a Boolean preview or a printability check. They do not appear in
renders/exports, create no objects and can be hidden with Show live preview.
Automatic sizing retains the 8 mm diameter / 6 mm length upper limits.

0.3.0: ARTICULATED HINGE
Choose Part A (two outer knuckles), Part B (middle knuckle), world axis X/Y/Z,
center coordinates or a scene Empty. Use 3D Cursor copies cursor coordinates.
Attachment direction 0 deg is +X for Z/Y axes, +Y for X axis.
Part A must be on the negative side, Part B on the positive side.
All dimensions are mm; center coordinates use scene length display units.

Defaults: pin 3 mm, radial clearance .25 mm, each axial gap .30 mm,
total hinge height 8 mm, wall 1.2 mm, swing each way 30 degrees.
Advanced: outer diameter, middle knuckle width, pin shaft length (0 = automatic).
Height includes all three knuckles and both axial gaps; head is additional.
At the defaults outer diameter is 5.9 mm. Each knuckle must be at least wall thick.

Preview: blue Part A, orange Part B, green pin; dashed lines show bevel boundaries.
The operator TRIMS mating faces with wedge/cylinder cuts to make room for motion,
then adds support bridges and knuckles. Larger swing angles remove more material.
Preview is a construction guide, not a full Boolean result.
Only selected parts are collision-tested at 5-degree samples and range endpoints;
this is not continuous collision analysis, a strength check or a print guarantee.
Closed connected manifold source/result meshes are required. Modifiers, shape
keys, linked meshes and overrides are rejected. Shared local meshes are preserved.
All results are prepared before commit; any build/commit error restores sources.
Ctrl+Z undoes the operation. Existing fixed connector functionality is retained.

Print A, B and headed pin SEPARATELY. Check supports beneath A's upper knuckle.
Clean holes, align parts, insert pin from the head side. Test rotation before
fixing. Secure exposed tip to PART A ONLY (for example with suitable adhesive).
Keep adhesive away from Part B and moving gaps. Head alone is not a two-way lock.
No snap-fit, print-in-place, automatic placement or interactive rotation tool yet.

Example: two 20 x 20 x 8 mm blocks centered at (-.01,0,0) and (.01,0,0) BU with
Unit Scale 1. Center (0,0,0), axis Z, direction 0 deg, remaining values default.
Detailed guide: https://github.com/marykaweb/JointCraft-3D/blob/main/docs/HINGE.md
