"""Marker prims and their materials.

Isaac-side only. Placement maths lives in whycode_sim.geometry so the scene and the
ground truth cannot disagree about where a marker is.
"""

import math

from pxr import Gf, Sdf, UsdGeom, UsdShade

from whycode_sim.geometry import marker_transform
from whycode_sim.isaac import usd_utils

MARKERS_ROOT = "/World/Markers"


def _quad_mesh(stage, path, size_m):
    """Single-sided square in local XY, normal along local +Z.

    UVs read upright when local +Y points at world up.
    """
    half = size_m / 2.0
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(
        [
            Gf.Vec3f(-half, -half, 0.0),
            Gf.Vec3f(half, -half, 0.0),
            Gf.Vec3f(half, half, 0.0),
            Gf.Vec3f(-half, half, 0.0),
        ]
    )
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0.0, 0.0, 1.0)] * 4)
    mesh.SetNormalsInterpolation("faceVarying")
    mesh.CreateExtentAttr([Gf.Vec3f(-half, -half, 0.0), Gf.Vec3f(half, half, 0.0)])

    primvars = UsdGeom.PrimvarsAPI(mesh)
    uvs = primvars.CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    uvs.Set(
        [Gf.Vec2f(0.0, 0.0), Gf.Vec2f(1.0, 0.0), Gf.Vec2f(1.0, 1.0), Gf.Vec2f(0.0, 1.0)]
    )
    return mesh


def _unlit_material(stage, path, texture_path, emissive_intensity):
    """OmniPBR configured to render the marker flat and lighting-independent.

    Mirrors the Gazebo models, which set albedo_map and emissive_map to the same texture
    to fake an unlit surface so detection did not depend on the lights.

    emissive_intensity is a calibration knob: RTX emission is in physical units, so a
    marker reads flat at values in the hundreds, not 1.0. Tune it in scene.yaml.
    """
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")

    texture = Sdf.AssetPath(str(texture_path))
    shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(texture)
    shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(1.0, 1.0, 1.0)
    )
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(
        1.0
    )
    shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
    shader.CreateInput("emissive_color_texture", Sdf.ValueTypeNames.Asset).Set(texture)
    shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(
        float(emissive_intensity)
    )

    material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    return material


def _post(stage, path, marker_position, marker_size, post_config):
    """Thin cylinder from the floor to the marker's lower edge."""
    lower_edge = marker_position[2] - marker_size / 2.0
    if lower_edge <= 0.01:
        return None

    post = UsdGeom.Cylinder.Define(stage, path)
    post.CreateRadiusAttr(float(post_config.radius))
    post.CreateHeightAttr(float(lower_edge))
    post.CreateAxisAttr("Z")
    post.CreateExtentAttr(
        [
            Gf.Vec3f(-post_config.radius, -post_config.radius, -lower_edge / 2.0),
            Gf.Vec3f(post_config.radius, post_config.radius, lower_edge / 2.0),
        ]
    )
    post.CreateDisplayColorAttr([Gf.Vec3f(*[float(c) for c in post_config.colour])])
    usd_utils.set_translate(
        post.GetPrim(), (marker_position[0], marker_position[1], lower_edge / 2.0)
    )
    return post


def build(stage, config, markers):
    """Create every marker in the scene. Returns prim paths in config order.

    Plain Xform + Mesh, no rigid body or collider: a marker with physics settles a
    millimetre on the first step, the same order as the error being measured.

    Paths are keyed by position in the layout rather than by id, because a layout may
    place the same id twice. Naming by id let the second one overwrite the first prim,
    leaving a marker in the ground truth that was not in the scene.
    """
    UsdGeom.Scope.Define(stage, MARKERS_ROOT)
    materials_root = f"{MARKERS_ROOT}/Materials"
    UsdGeom.Scope.Define(stage, materials_root)

    prim_paths = []
    materials = {}

    for index, marker in enumerate(markers):
        if not marker.texture_path.exists():
            raise FileNotFoundError(
                f"marker {marker.id} references {marker.texture_path}, which does not exist"
            )

        marker_path = f"{MARKERS_ROOT}/marker_{index:02d}_id{marker.id}"
        mesh = _quad_mesh(stage, marker_path, marker.size_m)

        transform = marker_transform(marker.position, marker.yaw_rad)
        usd_utils.set_transform(mesh.GetPrim(), Gf.Matrix4d(transform.T.tolist()))

        if marker.texture not in materials:
            materials[marker.texture] = _unlit_material(
                stage,
                f"{materials_root}/{marker.texture}",
                marker.texture_path,
                config.marker.emissive_intensity,
            )
        UsdShade.MaterialBindingAPI(mesh).Bind(materials[marker.texture])

        _post(
            stage,
            f"{marker_path}_post",
            marker.position,
            marker.size_m,
            config.marker.post,
        )
        prim_paths.append(marker_path)

    return prim_paths


def verify_placement(stage, markers, prim_paths, tolerance=1e-6):
    """Read every marker's world transform back and check it matches the config.

    The design rests on the scene and the ground-truth node deriving placement from the
    same YAML; this makes that a checked fact rather than an assumption.
    """
    from pxr import Usd

    if len(prim_paths) != len(markers):
        raise AssertionError(
            f"{len(markers)} markers configured but {len(prim_paths)} prims created"
        )

    for index, marker in enumerate(markers):
        prim = stage.GetPrimAtPath(prim_paths[index])
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        translation = matrix.ExtractTranslation()

        for axis, (actual, expected) in enumerate(zip(translation, marker.position)):
            if abs(actual - expected) > tolerance:
                raise AssertionError(
                    f"marker #{index} (id {marker.id}) sits at axis {axis} = {actual}, "
                    f"but the config says {expected}"
                )

        # The face normal is the third basis vector of the transform.
        normal = matrix.ExtractRotationMatrix().GetRow(2)
        expected_normal = (math.cos(marker.yaw_rad), math.sin(marker.yaw_rad), 0.0)
        for axis, (actual, expected) in enumerate(zip(normal, expected_normal)):
            if abs(actual - expected) > 1e-5:
                raise AssertionError(
                    f"marker #{index} (id {marker.id}) normal axis {axis} is {actual}, "
                    f"expected {expected}"
                )
