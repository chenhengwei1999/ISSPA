import open3d as o3d
import numpy as np


if __name__ == "__main__":
    # 读取 PLY 文件
    ply_file_path = "./data/point_cloud_outdoor/2024-06-25_11-20-02.ply"
    bin_file_path = "./data/point_cloud_outdoor/2024-06-25_11-20-02.bin"
    point_cloud = o3d.io.read_point_cloud(ply_file_path)

    pcd = np.fromfile(ply_file_path, dtype=np.float32)
    print(pcd.shape)
    print(pcd[:4], pcd[-100:-96])

    pcd_bin = np.fromfile(bin_file_path, dtype=np.float32)
    print(pcd_bin.shape)
    print(pcd_bin[8000:8004], pcd_bin[-5000:-4996])


    # 可视化点云
    o3d.visualization.draw_geometries([point_cloud])