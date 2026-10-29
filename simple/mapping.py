"""
UTS - 3D Mapping dengan Drone (CoppeliaSim, ZMQ Remote API) - Fast 3D Laser Scanner
Arena 10 x 10 m, drone mulai di tengah. Dummy 'target' berjalan mengikuti pola lawnmower,
drone mengikuti target, scanner merekam point cloud.

Dua sesi sapuan, ketinggian diatur di main():
  Sesi 1: jalur sejajar Y, tinggi height1
  Sesi 2: jalur sejajar X, tinggi height2 (jalur lebih rapat)

Jalankan (scene .ttt sudah terbuka di CoppeliaSim, jangan Play):
  python mapping.py              -> tinggi default (3.0 m dan 1.5 m)
  python mapping.py 3.5 1.2      -> sesi 1 di 3.5 m, sesi 2 di 1.2 m
Output: map_data.npz (cloud + path)
"""
import sys
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ================= KONFIGURASI =================
QUAD_PATH = '/Quadcopter'
TARGET_PATH = '/target'          # dummy yang diikuti script Quadcopter
SCANNER_PATH = '/fast3DLaserScanner'
X_RANGE = (-4.0, 4.0)            # arena 10x10 (+-5 m), margin 1 m dari dinding
Y_RANGE = (-4.0, 4.0)
SCAN_EVERY = 4                   # ambil scan tiap N step simulasi
PIX_STEP = 2                     # subsample piksel depth (1 = semua, lebih detail tapi lambat)
MIRROR_X = True                  # kalau peta tampak terbalik kiri-kanan -> False
COMPACT = 0.03                   # satu titik per kotak 3 cm (hemat memori)
OUT_FILE = 'map_data.npz'
# ===============================================

client = RemoteAPIClient()
sim = client.require('sim')
client.setStepping(True)

quad = sim.getObject(QUAD_PATH)
try:
    target = sim.getObject(TARGET_PATH)
except Exception:                # sebelum simulasi, target masih child dari base
    target = sim.getObject('/Quadcopter/base/target')

# Scanner adalah model terpisah dari Quadcopter pada hierarchy scene.
scanner = sim.getObject(SCANNER_PATH)
cams = sim.getObjectsInTree(scanner, sim.object_visionsensor_type, 0)
if not cams:
    raise SystemExit(f'Tidak ada vision sensor di bawah {SCANNER_PATH}. Pastikan model '
                     '"fast3DLaserScanner" ada di scene dan berisi vision sensor.')
for c in cams:
    print('vision sensor ditemukan di', SCANNER_PATH + ':', sim.getObjectAlias(c, 2))

clouds = []                      # kumpulan titik hasil scan
path = []                        # jalur drone


def set_pos(h, p):
    try:
        sim.setObjectPosition(h, -1, p)            # CoppeliaSim <= 4.6
    except Exception:
        sim.setObjectPosition(h, p, -1)            # CoppeliaSim >= 4.7


def get_pos(h):
    return sim.getObjectPosition(h, -1)


def lawnmower(height, lane_step, axis):
    """Waypoint lawnmower. axis='y': jalur sejajar Y, bergeser di X. axis='x': sebaliknya."""
    lane_rng, run_rng = (X_RANGE, Y_RANGE) if axis == 'y' else (Y_RANGE, X_RANGE)
    n = int(np.ceil((lane_rng[1] - lane_rng[0]) / lane_step)) + 1
    wps = []
    for i, c in enumerate(np.linspace(lane_rng[0], lane_rng[1], n)):
        a, b = (run_rng[0], run_rng[1]) if i % 2 == 0 else (run_rng[1], run_rng[0])
        wps += [(c, a, height), (c, b, height)] if axis == 'y' else [(a, c, height), (b, c, height)]
    return wps


def transit(pos, dest, safe_h):
    """Naik ke tinggi aman -> geser horizontal -> turun ke titik awal sapuan."""
    return [tuple(pos), (pos[0], pos[1], safe_h), (dest[0], dest[1], safe_h), tuple(dest)]


def densify(wps, ds):
    """Pecah segmen jadi titik berjarak ds, supaya target bergerak halus."""
    out = [np.array(wps[0], float)]
    for a, b in zip(wps[:-1], wps[1:]):
        a, b = np.array(a, float), np.array(b, float)
        L = np.linalg.norm(b - a)
        if L < 1e-6:
            continue
        m = max(1, int(np.ceil(L / ds)))
        for k in range(1, m + 1):
            out.append(a + (b - a) * k / m)
    return out


def scan_one(cam):
    """Baca depth (meter) satu sensor -> titik 3D dalam frame dunia."""
    depth, res = sim.getVisionSensorDepth(cam, 1)
    W, H = res
    d = np.frombuffer(depth, dtype=np.float32).reshape(H, W)
    fov = sim.getObjectFloatParam(cam, sim.visionfloatparam_perspective_angle)
    far = sim.getObjectFloatParam(cam, sim.visionfloatparam_far_clipping)
    f = (max(W, H) / 2.0) / np.tan(fov / 2.0)

    uu, vv = np.meshgrid(np.arange(0, W, PIX_STEP), np.arange(0, H, PIX_STEP))
    z = d[vv, uu]
    ok = (z > 0.05) & (z < far * 0.98)             # buang piksel tanpa hit (far clip)
    sx = -1.0 if MIRROR_X else 1.0
    x = sx * (uu - W / 2 + 0.5) / f * z
    y = (vv - H / 2 + 0.5) / f * z                 # baris 0 = bawah (konvensi CoppeliaSim)
    pc = np.stack([x[ok], y[ok], z[ok]], axis=1)

    M = np.array(sim.getObjectMatrix(cam, -1)).reshape(3, 4)
    return pc @ M[:, :3].T + M[:, 3]


def compact(pts, q=COMPACT):
    """Satu titik per kotak q x q x q (hapus duplikat)."""
    idx = np.floor(pts / q).astype(np.int64) + 20000
    key = (idx[:, 0] * 40000 + idx[:, 1]) * 40000 + idx[:, 2]
    _, u = np.unique(key, return_index=True)
    return pts[u]


def fly(wps, speed, dt, label, color):
    """Gerakkan target sepanjang wps dengan kecepatan konstan, scan sambil terbang."""
    traj = densify(wps, speed * dt)
    print(f'--- {label}: {len(traj)} step (~{len(traj) * dt:.0f} s simulasi)')
    try:                                           # jalur drone digambar di scene
        trail = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, 99999, color)
    except Exception:
        trail = None
    last = get_pos(quad)
    for k, p in enumerate(traj):
        set_pos(target, p.tolist())                # target berjalan sesuai pola
        client.step()
        if k % SCAN_EVERY == 0:
            clouds.append(np.concatenate([scan_one(c) for c in cams], axis=0).astype(np.float32))
            q = get_pos(quad)
            path.append(q)
            if trail is not None and np.linalg.norm(np.array(q) - np.array(last)) > 0.05:
                sim.addDrawingObjectItem(trail, list(last) + list(q))
                last = q
        if k % 200 == 0:
            print(f'    {100 * k / len(traj):5.1f}%')
    for _ in range(40):                            # biarkan drone menyusul target
        client.step()


def main(height1=4.0, height2=3.0):
    """
    height1 : ketinggian terbang sesi 1 (meter)
    height2 : ketinggian terbang sesi 2 (meter)
    Ubah di sini atau lewat argumen: python mapping.py <height1> <height2>
    """
    # (tinggi, jarak antar jalur, kecepatan target, arah jalur, warna jalur)
    sessions = [
        (height1, 1.6, 0.8, 'y', [0, 0, 1]),       # sesi 1: luas, jalur renggang
        (height2, 0.8, 0.5, 'x', [1, 0.5, 0]),     # sesi 2: detail, jalur rapat, silang
    ]
    safe_h = max(height1, height2)                 # tinggi aman saat berpindah titik

    sim.startSimulation()
    dt = sim.getSimulationTimeStep()
    start = get_pos(quad)
    set_pos(target, list(start))
    path.append(start)

    for i, (h, lane, spd, axis, color) in enumerate(sessions):
        wps = lawnmower(h, lane, axis)
        fly(transit(get_pos(quad), wps[0], safe_h) + wps[1:], spd, dt,
            f'Sesi {i + 1}: tinggi {h} m, jalur {axis.upper()}, lane {lane} m', color)

    sim.stopSimulation()

    cloud = compact(np.concatenate(clouds, axis=0))
    np.savez_compressed(OUT_FILE, cloud=cloud, path=np.array(path))
    print(f'Selesai. {len(cloud)} titik, {len(path)} titik jalur -> {OUT_FILE}')


if __name__ == '__main__':
    h1 = 4.0
    h2 = 3.0
    main(h1, h2)