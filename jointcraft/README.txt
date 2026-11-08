JointCraft 3D v0.2.1 — prototype, tested with Blender 5.2.0 LTS

Install ZIP: Edit > Preferences > Get Extensions > menu > Install from Disk.
Disable older separately installed JointCraft versions before installing.

Use:
1. Create two closed mesh boxes (e.g. 20 x 20 x 20 mm) sharing a face.
2. Object Mode > N panel > JointCraft.
3. Assign the pin object and socket object.
4. Click Create Auto Pin + Socket.
5. Inspect mesh and test fit before printing.

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
