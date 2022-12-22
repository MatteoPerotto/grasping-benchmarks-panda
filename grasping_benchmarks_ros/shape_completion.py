import numpy as np
import torch
from pathlib import Path

import open3d as o3d 

from pcr.model import PCRNetwork as Model
from pcr.utils import Normalize, Denormalize
from pcr.default_config import Config
from pcr.misc import download_checkpoint

def complete_point_cloud(partial_pc):

    ckpt_path = download_checkpoint(f'grasping.ckpt')

    model = Model(config=Config.Model)
    model.load_state_dict(torch.load(ckpt_path)['state_dict'])
    model.cuda()
    model.eval()

    # Normalize the point cloud
    partial, ctx = Normalize(Config.Processing)(partial_pc)

    partial = torch.tensor(partial, dtype=torch.float32).cuda().unsqueeze(0)
    complete, probabilities = model(partial)

    complete = complete.squeeze(0).cpu().numpy()
    partial = partial.squeeze(0).cpu().numpy()
    probabilities = probabilities.squeeze(0).cpu().numpy()

    # import ipdb; ipdb.set_trace()

    #root = Path('/home/panda-user')
    #pc_dir =  (root / 'points')
    #pc_dir.mkdir(exist_ok=True)

    #i = len(list(pc_dir.glob('*')))
    #np.save(pc_dir / f'reconstruction_{i}', np.concatenate([complete, probabilities[..., np.newaxis]], axis=1))
    #np.save(pc_dir / f'partial_{i}', partial)

    # Return the point cloud in its original reference frame
    complete = Denormalize(Config.Processing)(complete, ctx)
    partial = Denormalize(Config.Processing)(partial, ctx)

    o3d.visualization.draw([
        o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(partial)).paint_uniform_color([0, 1, 1]),
        o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(complete)).paint_uniform_color([1, 0, 1]),
        ],
        bg_color=(0,0,0,0))#,
        #show_skybox=False)

    # print(complete)
    return complete

# def test_version():

#     ckpt_path = download_checkpoint(f'grasping.ckpt')
#     asset_path = download_asset(f'partial_bleach_317.npy')

#     model = Model(config=Config.Model)
#     model.load_state_dict(torch.load(ckpt_path)['state_dict'])
#     model.cuda()
#     model.eval()

#     partial = np.load(asset_path)

#     partial, ctx = Normalize(Config.Processing)(partial)

#     partial = torch.tensor(partial, dtype=torch.float32).cuda().unsqueeze(0)
#     complete, probabilities = model(partial)

#     complete = complete.squeeze(0).cpu().numpy()
#     partial = partial.squeeze(0).cpu().numpy()

#     complete = Denormalize(Config.Processing)(complete, ctx)
#     partial = Denormalize(Config.Processing)(partial, ctx)

#     if visualize:
#         o3d.visualization.draw([
#               o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(partial)).paint_uniform_color([0, 0, 1]),
#               o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(complete)).paint_uniform_color([0, 1, 1]),
#               ])

#     print(complete)


# if __name__ == '__main__':
#     test_version()