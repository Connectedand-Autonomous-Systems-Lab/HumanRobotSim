# LiDAR Configuration Comparison

This experiment is run only once with:

- **Foveal angle**: 360°
- **Number of readings per scan**: 360  
- Purpose: To keep the **trajectory consistent** across all experiments.

LiDAR range is clipped to produce the following configurations:

| Foveal (°) | Number of Rays | SSIM   | Resizing Log |
|------------|----------------|--------|---------------|
| 360        | 360            | 1.0000 |               |
| 180        | 180            | 0.8160 | Resizing `map_180.pgm` from (2371, 1740) to match (2390, 1385) |
| 120        | 120            | 0.7966 | Resizing `map_120.pgm` from (2363, 1744) to match (2390, 1385) |
| 90         | 90             | 0.7888 | Resizing `map_90.pgm` from (2371, 1722) to match (2390, 1385)  |
| 60         | 60             | 0.7871 | Resizing `map_60.pgm` from (3526, 1405) to match (2390, 1385)  |

- **LiDAR resolution** across all runs is kept the same.
- **SSIM** is compared with the map produced by the **360° LiDAR scan**.
