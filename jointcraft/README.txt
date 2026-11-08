JointCraft 3D v0.2 — prototype for Blender 5.2

Install ZIP: Edit > Preferences > Get Extensions > menu > Install from Disk.
Disable the older JointCraft v0.1 before installing.

Use:
1. Create two closed mesh boxes (e.g. 20 x 20 x 20 mm) sharing a face.
2. Object Mode > N panel > JointCraft.
3. Assign the pin object and socket object.
4. Click Create Auto Pin + Socket.
5. Inspect mesh and test fit before printing.

The algorithm checks WORLD-AXIS-ALIGNED bounding-box face contact.
It is intended for boxes / flat-faced aligned parts, not arbitrary sculpted surfaces.
It calculates one fixed cylindrical pin and socket at the overlapping face center.
It does not yet support multiple joints, print-in-place hinges, face normals,
non-axis-aligned parts, or full printability validation.

Default clearance is RADIAL 0.20 mm (socket diameter exceeds pin diameter by 0.40 mm).
Wall setting 1.2 mm is used for conservative bounding-box dimension estimates,
NOT a guarantee of minimum wall thickness in arbitrary meshes.

Operations modify both meshes. Save a backup before applying. Ctrl+Z to undo.
