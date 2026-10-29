"""
Post process - hasil akhir 3D mapping (Grade A, kumulatif).
Membaca map_data.npz (hasil mapping.py) lalu menampilkan semua tahap dalam satu jendela:

  Panel 1  Scan 3D        : point cloud mentah                          (Grade B)
  Panel 2  Scan 2D        : peta datar tampak atas, hitam = rintangan   (Grade BC)
  Panel 3  Voxel + warna  : voxel dengan color map ketinggian + jalur   (Grade AB + A)
Jalur drone (Grade C) digambar biru di semua panel.

Fitur tambahan:
  - Deteksi objek otomatis: tiap rintangan dikotaki dan diukur (lebar, kedalaman, tinggi,
    luas alas, perkiraan volume) -> tabel di terminal + objek.csv
  - Ekspor point cloud berwarna ketinggian ke .ply (buka di MeshLab / CloudCompare)

Jalankan: python view_map.py        -> menyimpan hasil_akhir.png, objek.csv, cloud_berwarna.ply
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from objects import find_objects

VOXEL = 0.15                                   # ukuran voxel (m)
RES_2D = 0.1                                   # ukuran sel peta 2D (m)
OBST_Z = 0.25                                  # sel lebih tinggi dari ini = rintangan
MAX_RAW = 60000                                # batas titik mentah yang digambar (matplotlib berat)
BOUNDS = (-5.0, 5.0, -5.0, 5.0, -0.05, 2.5)   # xmin,xmax,ymin,ymax,zmin,zmax
OUT_PNG = 'hasil_akhir.png'
OUT_CSV = 'objek.csv'
OUT_PLY = 'cloud_berwarna.ply'

data = np.load('map_data.npz')
cloud, path = data['cloud'], data['path']
x0, x1, y0, y1, z0, z1 = BOUNDS

# ---- buang titik di luar arena
m = ((cloud[:, 0] > x0) & (cloud[:, 0] < x1) & (cloud[:, 1] > y0) &
     (cloud[:, 1] < y1) & (cloud[:, 2] > z0) & (cloud[:, 2] < z1))
cloud = cloud[m]
n_raw = len(cloud)

# ---- post process 1 (AB): titik -> voxel
idx = np.unique(np.floor(cloud / VOXEL).astype(np.int32), axis=0)
vox = (idx + 0.5) * VOXEL
reduksi = 100 * (1 - len(vox) / n_raw)
print(f'Titik mentah: {n_raw}  ->  voxel ({VOXEL} m): {len(vox)}  (reduksi {reduksi:.1f}%)')

# ---- scan 2D (BC): proyeksi atas, tiap sel menyimpan tinggi maksimum
nx, ny = int((x1 - x0) / RES_2D), int((y1 - y0) / RES_2D)
ix = np.clip(((cloud[:, 0] - x0) / RES_2D).astype(int), 0, nx - 1)
iy = np.clip(((cloud[:, 1] - y0) / RES_2D).astype(int), 0, ny - 1)
maxz = np.full((nx, ny), -np.inf)
np.maximum.at(maxz, (ix, iy), cloud[:, 2])
grid = np.full((nx, ny), 0.5)                          # abu-abu = belum terscan
grid[(maxz > -np.inf) & (maxz <= OBST_Z)] = 1.0        # putih = lantai / bebas
grid[maxz > OBST_Z] = 0.0                              # hitam = rintangan
print(f'Peta 2D {nx}x{ny}: rintangan {(grid == 0).sum()}, bebas {(grid == 1).sum()}, '
      f'belum terscan {(grid == 0.5).sum()} sel')

# ---- fitur tambahan: deteksi & pengukuran objek (lantai diasumsikan z = 0)
hmap = np.where(maxz > -np.inf, maxz, np.nan)
objs = find_objects(hmap, x0, y0, RES_2D, OBST_Z, min_cells=4)
print(f'\nObjek terdeteksi: {len(objs)}')
if objs:
    print(f'{"#":>2} {"pusat (x,y)":>14} {"lebar":>6} {"dalam":>6} {"tinggi":>7} {"alas m2":>8} {"vol m3":>7}')
    for k, o in enumerate(objs, 1):
        print(f'{k:>2} ({o["cx"]:+5.2f},{o["cy"]:+5.2f}) {o["width"]:6.2f} {o["depth"]:6.2f} '
              f'{o["height"]:7.2f} {o["area"]:8.2f} {o["volume"]:7.2f}')
    with open(OUT_CSV, 'w') as f:
        f.write('no,cx,cy,lebar_m,kedalaman_m,tinggi_m,luas_alas_m2,volume_m3\n')
        for k, o in enumerate(objs, 1):
            f.write(f'{k},{o["cx"]:.3f},{o["cy"]:.3f},{o["width"]:.3f},{o["depth"]:.3f},'
                    f'{o["height"]:.3f},{o["area"]:.3f},{o["volume"]:.3f}\n')
    print(f'Tabel objek disimpan: {OUT_CSV}')

# ---- fitur tambahan: ekspor .ply berwarna ketinggian
rgb = (plt.get_cmap('RdYlGn_r')(np.clip((cloud[:, 2] - z0) / (z1 - z0), 0, 1))[:, :3] * 255).astype(int)
with open(OUT_PLY, 'w') as f:
    f.write('ply\nformat ascii 1.0\n'
            f'element vertex {n_raw}\n'
            'property float x\nproperty float y\nproperty float z\n'
            'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
    np.savetxt(f, np.hstack([cloud, rgb]), fmt='%.4f %.4f %.4f %d %d %d')
print(f'Point cloud berwarna disimpan: {OUT_PLY}')


def box3d(ax, o):
    """Kotak pembatas objek (dari lantai sampai puncak) di panel 3D."""
    xs, ys, zt = (o['xmin'], o['xmax']), (o['ymin'], o['ymax']), o['height']
    for z in (0, zt):
        ax.plot([xs[0], xs[1], xs[1], xs[0], xs[0]], [ys[0], ys[0], ys[1], ys[1], ys[0]], [z] * 5,
                color='red', lw=1.2)
    for x in xs:
        for y in ys:
            ax.plot([x, x], [y, y], [0, zt], color='red', lw=1.2)


def style3d(ax, title):
    ax.plot(path[:, 0], path[:, 1], path[:, 2], color='blue', lw=1.2, label='Jalur drone')
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title(title)
    ax.set_box_aspect((x1 - x0, y1 - y0, (z1 - z0) * 1.5))
    ax.legend(loc='upper left')


fig = plt.figure(figsize=(18, 6.5))

# Panel 1: scan 3D mentah (B)
ax1 = fig.add_subplot(1, 3, 1, projection='3d')
raw = cloud if n_raw <= MAX_RAW else cloud[np.random.choice(n_raw, MAX_RAW, replace=False)]
ax1.scatter(raw[:, 0], raw[:, 1], raw[:, 2], c='gray', s=0.3)
style3d(ax1, f'Scan 3D - point cloud ({n_raw:,} titik)')

# Panel 2: scan 2D (BC)
ax2 = fig.add_subplot(1, 3, 2)
ax2.imshow(grid.T, origin='lower', extent=(x0, x1, y0, y1), cmap='gray', vmin=0, vmax=1)
ax2.plot(path[:, 0], path[:, 1], color='blue', lw=1.0, label='Jalur drone')
for k, o in enumerate(objs, 1):
    ax2.add_patch(Rectangle((o['xmin'], o['ymin']), o['width'], o['depth'], fill=False,
                            ec='red', lw=1.6))
    ax2.text(o['xmin'], o['ymax'] + 0.08, f"#{k}  {o['width']:.1f}x{o['depth']:.1f}x{o['height']:.1f} m",
             color='red', fontsize=8, weight='bold', backgroundcolor=(1, 1, 1, 0.6))
ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)')
ax2.set_title('Scan 2D - tampak atas + deteksi objek\n(hitam: rintangan, putih: bebas, abu: belum terscan)')
ax2.legend(loc='upper left')

# Panel 3: voxel + color map ketinggian (AB + A)
ax3 = fig.add_subplot(1, 3, 3, projection='3d')
sc = ax3.scatter(vox[:, 0], vox[:, 1], vox[:, 2], c=vox[:, 2], cmap='RdYlGn_r',
                 marker='s', s=6, depthshade=False)
fig.colorbar(sc, ax=ax3, shrink=0.6, label='Ketinggian Z (m)')
for o in objs:
    box3d(ax3, o)
style3d(ax3, f'Voxel {VOXEL} m + color map ketinggian ({len(vox):,} voxel)')

fig.tight_layout()
fig.savefig(OUT_PNG, dpi=150)
print(f'Disimpan: {OUT_PNG}')
plt.show()
