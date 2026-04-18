# mbes_ism

Inverse Sensor Model for Multibeam Echo Sounder (MBES). Converts raw MBES point clouds or laser scans into probabilistic point clouds for occupancy mapping.

## Dependencies

```bash
sudo apt-get install ros-jazzy-sensor-msgs ros-jazzy-std-msgs
```

---

## Nodes

### `mbes_ism` — `src/mbes_ism_node.cpp`

Applies a ray-casting inverse sensor model to each MBES beam. For every return, all range bins along the beam up to the hit are marked free (probability 0.1) and the hit bin is marked occupied (probability 0.9). Outputs a `PointCloud2` with intensity encoding occupancy probability, ready for ingestion into [iceberg_nav](https://github.com/GSO-soslab/iceberg_nav)'s `voxel_log_odds_visualizer`.

Supports both real hardware (`PointCloud2`) and Stonefish simulation (`LaserScan`) via the `stonefish` parameter.

**Subscribes**
| Type | Description |
|---|---|
| `PointCloud2` | Raw MBES point cloud (hardware) |
| `LaserScan` | Simulated MBES scan (Stonefish) |

**Publishes**
| Type | Description |
|---|---|
| `PointCloud2` | Probabilistic cloud (intensity = occupancy probability) |

---

## Launch Files

| Launch File | Description |
|---|---|
| `mbes_ism.launch.py` | Starts `mbes_ism` node |

---

## Configuration

### `config/mbes_ism.yaml`

| Parameter | Default | Description |
|---|---|---|
| `stonefish` | `false` | Simulation mode — subscribes to `LaserScan` instead of `PointCloud2` |
| `pointcloud_topic` | — | Input topic for hardware MBES point cloud |
| `laserscan_topic` | — | Input topic for simulated MBES scan |
| `pointcloud_pub_topic` | — | Output probabilistic point cloud topic |
| `range_resolution` | `1.0` | Bin size along each beam ray (m) |
