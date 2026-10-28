# Segmented human contact model v0.2.0

Entry points: `contact_model.usda` (animated USD skin and kinematic colliders) and `contact.xml` (MuJoCo contact rig, static visual skin). Original source files remain available.

19 convex body regions follow the source skeleton. Geometry is derived from dominant skin weights. Regions with no weighted vertices use an explicitly marked estimated fallback; inspect `contact_regions.json`. These are approximate rigid contact surfaces, not deformable skin or clothing. Hands are not separated into fingers. Mass, friction and contact-force limits are not calibrated.

The two-second sample moves the torso and forearms; it does not implement walking, balance or a sampling policy. Carried bags follow their attachment bone, filter wearer contact, and retain collision with external objects. The old navigation capsule is disabled in this entry point.

Use `tools/human_contact.py` with MuJoCo: construct `HumanContactRuntime(model, data, asset_dir)`, call `set_time(t)` before each simulation step, then `contacts(passenger_id)` to read region, position, normal and estimated force. The runtime uses asset-local coordinates; scene placement must transform the motion accordingly. Sampling tags are candidate geometric annotations, not SOP authorization.

Validation: all 19 regions have independent MuJoCo probe contacts plus a no-contact control; USD bone-follow transforms are checked at five frames; Isaac Sim tests an external head contact with all-body-colliders-disabled negative control. See `validation.json`. Older bag tests are explicitly retained under the legacy assembly key, not attributed to the new entry point.
