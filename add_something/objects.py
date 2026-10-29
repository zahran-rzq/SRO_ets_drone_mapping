"""
Deteksi & pengukuran objek dari peta tinggi 2D (dipakai mapping.py untuk live preview
dan view_map.py untuk hasil akhir). Tanpa dependensi tambahan (hanya numpy).

Alur: sel dengan tinggi > obst_z = rintangan -> tutup celah 1 sel -> kelompokkan sel yang
bertetangga (connected components, 8 arah) -> ukur tiap kelompok.
Asumsi: lantai berada di z = 0.
"""
import numpy as np


def _dilate(mask):
    """Perluas mask 1 sel ke 8 arah (untuk menutup celah kecil akibat sisi objek yang tak terscan)."""
    out = mask.copy()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            out[max(dx, 0):mask.shape[0] + min(dx, 0), max(dy, 0):mask.shape[1] + min(dy, 0)] |= \
                mask[max(-dx, 0):mask.shape[0] + min(-dx, 0), max(-dy, 0):mask.shape[1] + min(-dy, 0)]
    return out


def label_components(mask):
    """Beri label 1..n pada kelompok sel True yang bertetangga (8 arah). 0 = latar."""
    labels = np.zeros(mask.shape, int)
    n = 0
    nx, ny = mask.shape
    for sx, sy in zip(*np.nonzero(mask)):
        if labels[sx, sy]:
            continue
        n += 1
        labels[sx, sy] = n
        stack = [(sx, sy)]
        while stack:
            x, y = stack.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    a, b = x + dx, y + dy
                    if 0 <= a < nx and 0 <= b < ny and mask[a, b] and not labels[a, b]:
                        labels[a, b] = n
                        stack.append((a, b))
    return labels, n


def find_objects(hmap, x0, y0, res, obst_z=0.25, min_cells=4, wall_margin=4.6):
    """
    hmap[ix, iy] = tinggi (m) tiap sel, NaN = belum terscan. Sel (0,0) berpusat di (x0, y0)+res/2.
    Mengembalikan daftar objek (dict), diurutkan dari kiri ke kanan:
      xmin,xmax,ymin,ymax (m), width, depth, height (m), area (m2), volume (m3), cx, cy
    Sel di dekat dinding arena (|x| atau |y| > wall_margin) diabaikan.
    """
    nx, ny = hmap.shape
    cx = x0 + (np.arange(nx) + 0.5) * res
    cy = y0 + (np.arange(ny) + 0.5) * res
    inside = np.outer(np.abs(cx) <= wall_margin, np.abs(cy) <= wall_margin)
    h = np.nan_to_num(hmap, nan=-1.0)
    mask = (h > obst_z) & inside
    labels, n = label_components(_dilate(mask))
    objs = []
    for k in range(1, n + 1):
        cells = mask & (labels == k)
        if cells.sum() < min_cells:
            continue
        ix, iy = np.nonzero(cells)
        xmin, xmax = float(x0 + ix.min() * res), float(x0 + (ix.max() + 1) * res)
        ymin, ymax = float(y0 + iy.min() * res), float(y0 + (iy.max() + 1) * res)
        objs.append(dict(xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax,
                         width=xmax - xmin, depth=ymax - ymin,
                         height=float(h[cells].max()),
                         area=float(cells.sum() * res * res),
                         volume=float(h[cells].sum() * res * res),
                         cx=float(cx[ix].mean()), cy=float(cy[iy].mean())))
    objs.sort(key=lambda o: (o['cx'], o['cy']))
    return objs
