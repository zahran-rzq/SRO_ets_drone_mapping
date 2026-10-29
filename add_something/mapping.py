"""
UTS - 3D Mapping dengan Drone (CoppeliaSim, ZMQ Remote API) - Fast 3D Laser Scanner
Arena 10 x 10 m, drone mulai di tengah. Dummy 'target' berjalan mengikuti pola lawnmower,
drone mengikuti target, scanner merekam point cloud.

FITUR TAMBAHAN - LIVE PREVIEW + DETEKSI OBJEK: selama drone terbang, peta dibangun inkremental
dan ditampilkan real-time di jendela matplotlib:
  kiri : peta 2D tampak atas, warna = tinggi, abu-abu = belum terscan, titik biru = drone,
         kotak merah = objek yang terdeteksi otomatis (nomor + perkiraan tinggi)
  kanan: peta 3D voxel berwarna tinggi + jalur drone
  judul: posisi drone, jumlah titik/voxel/objek, persentase cakupan arena
Di akhir, gambar preview disimpan ke preview_final.png.

Dua sesi sapuan, ketinggian diatur di main():
  Sesi 1: jalur sejajar Y, tinggi height1
  Sesi 2: jalur sejajar X, tinggi height2 (jalur lebih rapat)

Jalankan (scene .ttt sudah terbuka di CoppeliaSim, jangan Play):
  python mapping.py              -> tinggi default (4.0 m dan 3.0 m)
  python mapping.py 3.5 1.2      -> sesi 1 di 3.5 m, sesi 2 di 1.2 m
Output: map_data.npz (cloud + path), preview_final.png
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from objects import find_objects

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

# --- live preview
PREVIEW = True                   # False = tanpa jendela preview
PREVIEW_EVERY = 5                # perbarui preview tiap N scan (naikkan kalau terasa lambat)
VOXEL = 0.15                     # ukuran voxel untuk preview (m)
BOUNDS = (-5.0, 5.0, -5.0, 5.0, -0.05, 2.5)   # xmin,xmax,ymin,ymax,zmin,zmax
MAX_3D_VOXELS = 12000            # batas voxel yang digambar di panel 3D (agar tetap lancar)
COVER_LIM = 4.5                  # cakupan dihitung di |x|,|y| <= nilai ini
PREVIEW_PNG = 'preview_final.png'
OBST_Z = 0.25                    # sel lebih tinggi dari ini dianggap rintangan (deteksi objek)
OBJ_MIN_CELLS = 3                # objek minimal terdiri dari N sel voxel 2D
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
state = {'scans': 0, 'points': 0, 'checked': False}
preview = None

# ---------------- grid voxel inkremental (untuk preview) ----------------
ORIGIN = np.array([BOUNDS[0], BOUNDS[2], BOUNDS[4]])
DIMS = np.ceil((np.array([BOUNDS[1], BOUNDS[3], BOUNDS[5]]) - ORIGIN) / VOXEL).astype(int)
grid = np.zeros(DIMS, bool)      # True = voxel terisi
_cx = ORIGIN[0] + (np.arange(DIMS[0]) + 0.5) * VOXEL
_cy = ORIGIN[1] + (np.arange(DIMS[1]) + 0.5) * VOXEL
REGION = np.outer(np.abs(_cx) <= COVER_LIM, np.abs(_cy) <= COVER_LIM)   # area penilaian cakupan


def voxel_update(pts):
    """Tandai voxel yang berisi titik baru."""
    i = np.floor((pts - ORIGIN) / VOXEL).astype(int)
    i = i[np.all((i >= 0) & (i < DIMS), axis=1)]
    grid[i[:, 0], i[:, 1], i[:, 2]] = True


class LivePreview:
    """Jendela real-time: peta 2D tinggi (kiri) + peta 3D voxel berwarna (kanan)."""

    def __init__(self):
        plt.ion()
        self.fig = plt.figure(figsize=(14, 6.5))
        z0, z1 = BOUNDS[4], BOUNDS[5]
        cmap = plt.get_cmap('RdYlGn_r').copy()
        cmap.set_bad('0.88')                                   # abu-abu = belum terscan

        self.ax2 = self.fig.add_subplot(1, 2, 1)
        self.img = self.ax2.imshow(np.full((DIMS[1], DIMS[0]), np.nan), origin='lower',
                                   extent=(BOUNDS[0], BOUNDS[1], BOUNDS[2], BOUNDS[3]),
                                   cmap=cmap, vmin=z0, vmax=z1)
        self.l2, = self.ax2.plot([], [], color='blue', lw=1.2)
        self.d2, = self.ax2.plot([], [], 'o', color='blue', ms=8, mec='white')
        self.box_artists = []
        self.ax2.set_title('Peta 2D tampak atas (warna = tinggi, kotak merah = objek)')
        self.ax2.set_xlabel('X (m)'); self.ax2.set_ylabel('Y (m)')

        self.ax3 = self.fig.add_subplot(1, 2, 2, projection='3d')
        self.sc = self.ax3.scatter([0], [0], [0], c=[0], cmap=cmap, vmin=z0, vmax=z1,
                                   s=6, marker='s', depthshade=False)
        self.l3, = self.ax3.plot([], [], [], color='blue', lw=1.2)
        self.ax3.set_xlim(BOUNDS[0], BOUNDS[1]); self.ax3.set_ylim(BOUNDS[2], BOUNDS[3])
        self.ax3.set_zlim(z0, z1)
        self.ax3.set_box_aspect((BOUNDS[1] - BOUNDS[0], BOUNDS[3] - BOUNDS[2], (z1 - z0) * 1.5))
        self.ax3.set_xlabel('X'); self.ax3.set_ylabel('Y'); self.ax3.set_zlabel('Z')
        self.ax3.set_title('Peta 3D voxel')
        self.fig.colorbar(self.sc, ax=self.ax3, shrink=0.6, label='Ketinggian Z (m)')
        self.fig.tight_layout(rect=(0, 0, 1, 0.9))
        plt.show(block=False)

    def update(self, label, q):
        occ = grid.any(axis=2)
        top = (grid * np.arange(DIMS[2])).max(axis=2)          # indeks voxel tertinggi per kolom
        h = np.where(occ, ORIGIN[2] + (top + 0.5) * VOXEL, np.nan)
        self.img.set_data(np.ma.masked_invalid(h).T)

        for a in self.box_artists:                              # gambar ulang kotak objek
            a.remove()
        self.box_artists = []
        objs = find_objects(h, ORIGIN[0], ORIGIN[1], VOXEL, OBST_Z, OBJ_MIN_CELLS)
        for k, o in enumerate(objs, 1):
            r = Rectangle((o['xmin'], o['ymin']), o['width'], o['depth'], fill=False,
                          ec='red', lw=1.6)
            t = self.ax2.text(o['xmin'], o['ymax'] + 0.05, f"#{k} h~{o['height']:.1f} m",
                              color='red', fontsize=8, weight='bold')
            self.ax2.add_patch(r)
            self.box_artists += [r, t]

        p = np.array(path)
        self.l2.set_data(p[:, 0], p[:, 1])
        self.d2.set_data([q[0]], [q[1]])

        ix, iy, iz = np.nonzero(grid)
        if len(ix) > MAX_3D_VOXELS:
            s = np.random.choice(len(ix), MAX_3D_VOXELS, replace=False)
            ix, iy, iz = ix[s], iy[s], iz[s]
        if len(ix):
            x = ORIGIN[0] + (ix + 0.5) * VOXEL
            y = ORIGIN[1] + (iy + 0.5) * VOXEL
            z = ORIGIN[2] + (iz + 0.5) * VOXEL
            self.sc._offsets3d = (x, y, z)
            self.sc.set_array(z)
        self.l3.set_data(p[:, 0], p[:, 1])
        self.l3.set_3d_properties(p[:, 2])

        cover = 100.0 * occ[REGION].mean()
        self.fig.suptitle(f'{label}\ndrone ({q[0]:+.1f}, {q[1]:+.1f}, {q[2]:.1f}) m  |  '
                          f'titik {state["points"]:,}  |  voxel {int(grid.sum()):,}  |  objek {len(objs)}  |  '
                          f'cakupan arena {cover:.1f}%', fontsize=11)
        self.fig.canvas.draw_idle()
        plt.pause(0.001)


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


def check_scanner_follows(q):
    """Peringatan sekali: scanner harus ikut terbang bersama drone."""
    if state['checked'] or np.linalg.norm(np.array(q) - np.array(path[0])) < 1.0:
        return
    state['checked'] = True
    gap = np.linalg.norm(np.array(get_pos(scanner)) - np.array(q))
    if gap > 0.6:
        print(f'!! PERINGATAN: scanner berjarak {gap:.1f} m dari drone. Jadikan {SCANNER_PATH} '
              'child dari Quadcopter/base supaya ikut bergerak, kalau tidak peta akan salah.')


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
            pts = np.concatenate([scan_one(c) for c in cams], axis=0).astype(np.float32)
            clouds.append(pts)
            voxel_update(pts)
            state['scans'] += 1
            state['points'] += len(pts)
            q = get_pos(quad)
            path.append(q)
            check_scanner_follows(q)
            if trail is not None and np.linalg.norm(np.array(q) - np.array(last)) > 0.05:
                sim.addDrawingObjectItem(trail, list(last) + list(q))
                last = q
            if preview is not None and state['scans'] % PREVIEW_EVERY == 0:
                preview.update(label, q)
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
    global preview
    # (tinggi, jarak antar jalur, kecepatan target, arah jalur, warna jalur)
    sessions = [
        (height1, 1.6, 0.8, 'y', [0, 0, 1]),       # sesi 1: luas, jalur renggang
        (height2, 0.8, 0.5, 'x', [1, 0.5, 0]),     # sesi 2: detail, jalur rapat, silang
    ]
    safe_h = max(height1, height2)                 # tinggi aman saat berpindah titik

    if PREVIEW:
        preview = LivePreview()

    sim.startSimulation()
    dt = sim.getSimulationTimeStep()
    start = get_pos(quad)
    set_pos(target, list(start))
    path.append(start)

    label = ''
    for i, (h, lane, spd, axis, color) in enumerate(sessions):
        wps = lawnmower(h, lane, axis)
        label = f'Sesi {i + 1}: tinggi {h} m, jalur {axis.upper()}, lane {lane} m'
        fly(transit(get_pos(quad), wps[0], safe_h) + wps[1:], spd, dt, label, color)

    sim.stopSimulation()

    cloud = compact(np.concatenate(clouds, axis=0))
    np.savez_compressed(OUT_FILE, cloud=cloud, path=np.array(path))
    print(f'Selesai. {len(cloud)} titik, {len(path)} titik jalur -> {OUT_FILE}')

    if preview is not None:                        # tampilan akhir + simpan gambar untuk laporan
        preview.update('SELESAI - ' + label, path[-1])
        preview.fig.savefig(PREVIEW_PNG, dpi=150)
        print(f'Preview akhir disimpan: {PREVIEW_PNG} (tutup jendela untuk keluar)')
        plt.ioff()
        plt.show()


if __name__ == '__main__':
    h1 = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
    h2 = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    main(h1, h2)
