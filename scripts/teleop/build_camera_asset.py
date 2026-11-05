"""Build a USD overlay from the official SO101 camera meshes; run with Isaac USD Python libs."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[2]


def build():
    folder = ROOT / 'Assets/Robots/soarm101'
    source = folder / 'camera_source'
    urdf = ET.parse(source / 'so101_new_calib_camera.urdf').getroot()
    original = Usd.Stage.Open(str(folder / 'SO101.usd'))
    root_path = str(original.GetDefaultPrim().GetPath())
    output = folder / 'SO101_camera.usda'
    stage = Usd.Stage.CreateNew(str(output))
    root = stage.DefinePrim(root_path, 'Xform')
    root.GetReferences().AddReference('./SO101.usd')
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    paths = {'gripper_link': root_path + '/gripper_link'}
    for joint_name in ('wrist_camera_mount_joint', 'wrist_camera_joint'):
        joint = urdf.find(f"joint[@name='{joint_name}']")
        child = joint.find('child').get('link')
        path = paths[joint.find('parent').get('link')] + '/' + child
        paths[child] = path
        xform = UsdGeom.Xform.Define(stage, path)
        origin = joint.find('origin')
        xyz = list(map(float, origin.get('xyz').split()))
        quat = Rotation.from_euler('xyz', list(map(float, origin.get('rpy').split()))).as_quat()
        xform.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        xform.AddOrientOp().Set(Gf.Quatf(float(quat[3]), Gf.Vec3f(*map(float, quat[:3]))))
        visual = urdf.find(f"link[@name='{child}']/visual")
        mesh_info = visual.find('geometry/mesh')
        scale = np.array(list(map(float, mesh_info.get('scale', '1 1 1').split())))
        data = (source / Path(mesh_info.get('filename')).name).read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        if len(data) != 84 + count * 50:
            raise ValueError('Expected binary STL')
        records = np.frombuffer(data, dtype=np.dtype([('normal','<f4',(3,)), ('vertices','<f4',(3,3)), ('attribute','<u2')]), offset=84)
        vertices = records['vertices'].reshape(-1, 3) * scale
        mesh = UsdGeom.Mesh.Define(stage, path + '/visual')
        mesh.CreatePointsAttr(vertices.tolist())
        mesh.CreateFaceVertexCountsAttr([3] * count)
        mesh.CreateFaceVertexIndicesAttr(list(range(count * 3)))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([(0.75, 0.75, 0.78) if 'mount' in child else (0.08, 0.08, 0.08)])
    # Visual payload only: preserve the calibrated articulation and its dynamics.
    # Camera intrinsics/extrinsics live in env_cfg/camera/soarm101.yml.
    stage.GetRootLayer().Save()
    print(output)


if __name__ == '__main__':
    build()
