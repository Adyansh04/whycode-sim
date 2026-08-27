"""Small USD helpers.

Referenced assets arrive with xformOpOrder already populated, so AddTranslateOp raises
rather than moving them. These set an existing op when there is one and add it otherwise.
"""

from pxr import Gf, UsdGeom


def _find_op(xformable, op_type, op_suffix=""):
    """Existing xformOp of the given type on this prim, or None."""
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == op_type and op.SplitName()[-1] == (
            op_suffix if op_suffix else op.SplitName()[-1]
        ):
            return op
    return None


def set_translate(prim, translation):
    """Set the prim's local translation, reusing an existing op when present."""
    xformable = UsdGeom.Xformable(prim)
    value = Gf.Vec3d(*[float(v) for v in translation])
    op = _find_op(xformable, UsdGeom.XformOp.TypeTranslate)
    if op is None:
        op = xformable.AddTranslateOp()
    op.Set(value)
    return op


def set_orient(prim, quaternion):
    """Set local orientation from a Gf.Quatd/Quatf, reusing an existing op.

    Handles both orient (quaternion) and rotateXYZ (euler) authoring, since which an
    asset uses is not up to us.
    """
    xformable = UsdGeom.Xformable(prim)

    op = _find_op(xformable, UsdGeom.XformOp.TypeOrient)
    if op is not None:
        precision = op.GetPrecision()
        if precision == UsdGeom.XformOp.PrecisionDouble:
            op.Set(Gf.Quatd(quaternion))
        else:
            op.Set(Gf.Quatf(quaternion))
        return op

    euler = _find_op(xformable, UsdGeom.XformOp.TypeRotateXYZ)
    if euler is not None:
        rotation = Gf.Rotation(Gf.Quatd(quaternion))
        angles = rotation.Decompose(
            Gf.Vec3d(0, 0, 1), Gf.Vec3d(0, 1, 0), Gf.Vec3d(1, 0, 0)
        )
        euler.Set(Gf.Vec3f(float(angles[2]), float(angles[1]), float(angles[0])))
        return euler

    return xformable.AddOrientOp().Set(Gf.Quatf(quaternion))


def set_transform(prim, matrix4d):
    """Replace a prim's local transform with a single matrix op.

    Clears the existing op order, so use it on prims we author, not referenced assets.
    """
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(matrix4d)
