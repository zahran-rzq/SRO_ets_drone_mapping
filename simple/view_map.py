"""
Post process hasil mapping (map_data.npz).
  python view_map.py raw     -> point cloud mentah            (Grade B)
  python view_map.py voxel   -> voxel grid 1 warna            (Grade AB)
  python view_map.py color   -> voxel + color map ketinggian  (Grade A)
Jalur drone digambar biru.
"""
import sys
import numpy as np
import matplotlib.pyplot as plt

MODE = sys.argv[1] if len(sys.argv) > 1 else 'color'
VOXEL = 0.15                                   # ukuran voxel (m)
BOUNDS = (-5.0, 5.0, -5.0, 5.0, -0.05, 2.5)   # xmin,xmax,ymin,ymax,zmin,zmax

data = np.load('map_data.npz')
cloud, path = data['cloud'], data['path']

# buang titik di luar arena
x0, x1, y0, y1, z0, z1 = BOUNDS
m = ((cloud[:, 0] > x0) & (cloud[:, 0] < x1) & (cloud[:, 1] > y0) &
     (cloud[:, 1] < y1) & (cloud[:, 2] > z0) & (cloud[:, 2] < z1))
cloud = cloud[m]
print(f'Titik mentah: {len(cloud)}')

if MODE == 'raw':
    pts = cloud
    if len(pts) > 150000:                      # matplotlib berat untuk jutaan titik
        pts = pts[np.random.choice(len(pts), 150000, replace=False)]
else:
    idx = np.unique(np.floor(cloud / VOXEL).astype(np.int32), axis=0)
    pts = (idx + 0.5) * VOXEL
    print(f'Voxel ({VOXEL} m): {len(pts)}  (reduksi {100 * (1 - len(pts) / len(cloud)):.1f}%)')

fig = plt.figure(figsize=(9, 8))
ax = fig.add_subplot(111, projection='3d')
if MODE == 'color':
    sc = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=pts[:, 2], cmap='RdYlGn_r',
                    marker='s', s=6, depthshade=False)
    fig.colorbar(sc, ax=ax, shrink=0.6, label='Ketinggian Z (m)')
elif MODE == 'voxel':
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='tab:green', marker='s', s=6, depthshade=False)
else:
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='gray', s=0.5)

ax.plot(path[:, 0], path[:, 1], path[:, 2], color='blue', lw=1.5, label='Jalur drone')
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_title(f'3D Map ({MODE})')
ax.set_box_aspect((x1 - x0, y1 - y0, (z1 - z0) * 1.5))
ax.legend()
plt.show()