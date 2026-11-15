"""3D view of the real RoboMaster EP (DJI meshes from robomaster_description) for the demo video.

The robot model is the URDF the driver uses (robomaster_ep.urdf), posed with the arm joint angles read from the robot
at the saved camera-arm pose (ep_joint_states_saved_arm_pose.json). Each frame only moves the whole robot to the
EKF pose (x, y, heading); paths, the uncertainty ellipse and camera rays are drawn on top by projecting their 3D
points with the same camera, so they stay exactly on the floor.

Run with the video venv:  env -u PYTHONPATH ~/phase3_video_venv/bin/python video_tools/ep3d.py   (writes a test PNG)
"""
import json
import math
import os
from pathlib import Path

os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')

import cv2                 # noqa: E402
import numpy as np         # noqa: E402
import pyrender            # noqa: E402
import trimesh             # noqa: E402
import yourdfpy            # noqa: E402

HERE = Path(__file__).resolve().parent
DESC = Path('~/ros2_ws/src/robomaster_ros/robomaster_description').expanduser()


def _mesh_path(fname):
    return str(DESC / fname.replace('package://robomaster_description/', ''))


def look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = map(np.asarray, (eye, target, up))
    z = eye - target
    z = z / np.linalg.norm(z)
    x = np.cross(up, z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, eye
    return T


def pose2d(x, y, th, z=0.0):
    T = np.eye(4)
    c, s = math.cos(th), math.sin(th)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = [x, y, z]
    return T


def _floor_texture(x0, x1, y0, y1, px_per_m=400):
    w, h = int((x1 - x0) * px_per_m), int((y1 - y0) * px_per_m)
    img = np.full((h, w, 3), (13, 19, 29), np.uint8)
    for gx in np.arange(math.ceil(x0 * 10) / 10, x1, 0.1):
        u = int((gx - x0) * px_per_m)
        major = abs(gx - round(gx * 2) / 2) < 1e-6
        cv2.line(img, (u, 0), (u, h), (46, 62, 84) if major else (27, 37, 52), 3 if major else 1)
    for gy in np.arange(math.ceil(y0 * 10) / 10, y1, 0.1):
        v = int((y1 - gy) * px_per_m)
        major = abs(gy - round(gy * 2) / 2) < 1e-6
        cv2.line(img, (0, v), (w, v), (46, 62, 84) if major else (27, 37, 52), 3 if major else 1)
    return img


def _textured_quad(x0, x1, y0, y1, z, img_rgb):
    verts = np.array([[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]])
    faces = np.array([[0, 1, 2], [0, 2, 3]])
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)
    from PIL import Image
    vis = trimesh.visual.TextureVisuals(uv=uv, image=Image.fromarray(img_rgb))
    return trimesh.Trimesh(vertices=verts, faces=faces, visual=vis, process=False)


class EPScene:
    def __init__(self, width=960, height=540, tag_x=1.36, tag_size=0.154, front=0.16, stop=0.50,
                 eye=(-0.60, -0.70, 0.85), target=(0.50, 0.0, 0.02), yfov_deg=40.0):
        self.w, self.h = width, height
        self.tag_x, self.front, self.stop = tag_x, front, stop
        urdf = yourdfpy.URDF.load(str(HERE / 'robomaster_ep.urdf'), filename_handler=_mesh_path,
                                  build_scene_graph=True, load_meshes=True)
        joints = json.loads((HERE / 'ep_joint_states_saved_arm_pose.json').read_text())
        urdf.update_cfg({k: v for k, v in joints.items() if k in urdf.actuated_joint_names})
        self.cam_offset = urdf.get_transform('camera_optical_link', 'base_link')[:3, 3].copy()   # real camera

        self.scene = pyrender.Scene(bg_color=[0.027, 0.043, 0.071, 1.0], ambient_light=[0.35, 0.35, 0.38])
        self.robot = pyrender.Node(name='robot', matrix=np.eye(4))
        self.scene.add_node(self.robot)
        for geom in urdf.scene.dump(concatenate=False):
            if 'collision' in str(getattr(geom, 'metadata', {}).get('name', '')):
                continue
            try:
                m = pyrender.Mesh.from_trimesh(geom, smooth=False)
            except Exception:
                geom.visual = trimesh.visual.ColorVisuals(geom, face_colors=[60, 64, 70, 255])
                m = pyrender.Mesh.from_trimesh(geom, smooth=False)
            self.scene.add(m, parent_node=self.robot)

        # floor with a 10 cm grid, the tag lying flat, start and stop lines
        fx0, fx1, fy0, fy1 = -0.6, 1.8, -0.8, 0.8
        self.static = []
        self.static.append(self.scene.add(pyrender.Mesh.from_trimesh(
            _textured_quad(fx0, fx1, fy0, fy1, 0.0, _floor_texture(fx0, fx1, fy0, fy1)), smooth=False)))
        tag_img = cv2.cvtColor(cv2.imread(str(HERE / 'tag36h11_id3.png')), cv2.COLOR_BGR2RGB)
        tag_img = cv2.copyMakeBorder(tag_img, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=(255, 255, 255))
        half = tag_size * (10 / 8) / 2 + 0.02          # black square + white border + paper margin
        self.static.append(self.scene.add(pyrender.Mesh.from_trimesh(
            _textured_quad(tag_x - half, tag_x + half, -half, half, 0.002, np.ascontiguousarray(tag_img[::-1])),
            smooth=False)))

        cam = pyrender.PerspectiveCamera(yfov=math.radians(yfov_deg), aspectRatio=width / height)
        self.eye0, self.target0 = np.array(eye, float), np.array(target, float)
        self.cam_node = self.scene.add(cam, pose=look_at(eye, target))
        self.P = cam.get_projection_matrix(width, height)
        self.set_camera(eye, target)
        key = pyrender.DirectionalLight(color=np.ones(3), intensity=3.2)
        self.scene.add(key, pose=look_at((-0.5, -1.2, 2.0), (0.4, 0, 0)))
        fill = pyrender.DirectionalLight(color=np.ones(3), intensity=1.4)
        self.scene.add(fill, pose=look_at((1.5, 1.2, 1.5), (0.4, 0, 0)))
        self.renderer = pyrender.OffscreenRenderer(width, height)

    def set_camera(self, eye, target):
        self.cam_pose = look_at(eye, target)
        self.scene.set_pose(self.cam_node, self.cam_pose)
        self.V = np.linalg.inv(self.cam_pose)

    def follow(self, x):
        """Chase camera: same viewing angle, eye and target slide along the path with the robot."""
        shift = np.array([x, 0.0, 0.0])
        self.set_camera(self.eye0 + shift, self.target0 + shift)

    def robot_mask(self):
        for n in self.static:
            n.mesh.is_visible = False
        depth = self.renderer.render(self.scene, flags=pyrender.RenderFlags.DEPTH_ONLY)
        for n in self.static:
            n.mesh.is_visible = True
        return depth > 0

    # -------------------------------------------------------------- projection helpers
    def project(self, pts):
        pts = np.atleast_2d(np.asarray(pts, float))
        if pts.shape[1] == 2:
            pts = np.hstack([pts, np.zeros((len(pts), 1))])
        hom = np.hstack([pts, np.ones((len(pts), 1))]) @ (self.P @ self.V).T
        ndc = hom[:, :3] / hom[:, 3:4]
        u = (ndc[:, 0] + 1) / 2 * self.w
        v = (1 - ndc[:, 1]) / 2 * self.h
        return np.stack([u, v], 1)

    def _poly(self, img, pts, color, thick=3, closed=False):
        p = (self.project(pts) * 16).astype(np.int32)
        cv2.polylines(img, [p], closed, color, thick, cv2.LINE_AA, shift=4)

    def _fill(self, img, pts, color, alpha):
        over = img.copy()
        cv2.fillPoly(over, [(self.project(pts) * 16).astype(np.int32)], color, cv2.LINE_AA, shift=4)
        cv2.addWeighted(over, alpha, img, 1 - alpha, 0, dst=img)

    def _label(self, img, xyz, text, color, scale=0.55):
        from textdraw import draw_text
        u, v = self.project([xyz])[0]
        draw_text(img, (u, v), text, color, size=int(44 * scale * self.w / 1280), weight='Bold')

    # -------------------------------------------------------------- one frame
    def render(self, x, y, th, ekf_path=None, odom_path=None, P2=None, ray=None, ray_ok=True, ell_scale=5.0,
               follow=True, decor=True):
        """RGB image. ekf_path/odom_path: (N,2) floor points; P2: 2x2 position covariance; ray: camera-fix strength
        0..1 (fades out). Floor drawings go under the robot (robot pixels restored from a robot-only depth mask)."""
        self.scene.set_pose(self.robot, pose2d(x, y, th))
        if follow:
            self.follow(x)
        color, _ = self.renderer.render(self.scene)
        base = np.ascontiguousarray(color[:, :, :3]).copy()
        mask = self.robot_mask()
        img = base.copy()
        if not decor:
            return img
        # start / stop lines across the path (chassis front positions)
        for xl, col, lab in [(self.front, (165, 180, 190), 'START 1.20 m'),
                             (self.tag_x - self.stop, (180, 140, 255), 'STOP 0.50 m')]:
            self._poly(img, [(xl, -0.40, 0.003), (xl, 0.40, 0.003)], col, 2)
            self._label(img, (xl + 0.015, -0.42, 0.0), lab, col)
        # camera field of view on the floor
        cx, cy = x + self.front * math.cos(th), y + self.front * math.sin(th)
        half, r = math.radians(45), 0.6
        cone = [(cx, cy), (cx + r * math.cos(th + half), cy + r * math.sin(th + half)),
                (cx + r * math.cos(th - half), cy + r * math.sin(th - half))]
        self._fill(img, cone, (255, 181, 71), 0.10)
        # paths
        if odom_path is not None and len(odom_path) > 1:
            self._poly(img, np.c_[odom_path, np.full(len(odom_path), 0.004)], (61, 220, 151), 3)
        if ekf_path is not None and len(ekf_path) > 1:
            self._poly(img, np.c_[ekf_path, np.full(len(ekf_path), 0.005)], (77, 163, 255), 4)
        # 2-sigma position ellipse (exaggerated)
        if P2 is not None and np.all(np.isfinite(P2)):
            vals, vecs = np.linalg.eigh(P2)
            vals = np.sqrt(np.clip(vals, 1e-12, None)) * 2 * ell_scale
            a = np.linspace(0, 2 * math.pi, 64)
            pts = (vecs @ np.vstack([vals[0] * np.cos(a), vals[1] * np.sin(a)])).T + [x, y]
            pts3 = np.c_[pts, np.full(len(pts), 0.006)]
            self._fill(img, pts3, (77, 163, 255), 0.22)
            self._poly(img, pts3, (77, 163, 255), 2, closed=True)
        img[mask] = base[mask]                    # everything above is on the floor, under the robot
        # camera fix: ray from the camera to the tag (in front of everything)
        if ray is not None:
            o = self.cam_offset
            cam3 = (x + o[0] * math.cos(th) - o[1] * math.sin(th), y + o[0] * math.sin(th) + o[1] * math.cos(th), o[2])
            col = (255, 181, 71) if ray_ok else (255, 93, 108)
            ov = img.copy()
            self._poly(ov, [cam3, (self.tag_x, 0.0, 0.002)], col, 4)
            cv2.addWeighted(ov, ray, img, 1 - ray, 0, dst=img)
        return img


if __name__ == '__main__':
    import time
    t0 = time.time()
    sc = EPScene()
    print(f'scene built in {time.time() - t0:.1f} s')
    path = np.c_[np.linspace(0, 0.45, 30), 0.01 * np.sin(np.linspace(0, 3, 30))]
    odo = np.c_[np.linspace(0, 0.5, 30), 0.03 * np.linspace(0, 1, 30)]
    t0 = time.time()
    img = sc.render(0.45, 0.01, 0.03, ekf_path=path, odom_path=odo, P2=np.diag([0.008 ** 2, 0.015 ** 2]), ray=0.8)
    print(f'frame in {time.time() - t0:.2f} s')
    out = Path(os.environ.get('OUT', '/tmp')) / 'ep3d_test.png'
    cv2.imwrite(str(out), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    print(out)
