import os
import argparse
from isaacsim import SimulationApp

# Parse command line arguments
def parse_args():
    parser = argparse.ArgumentParser(description='LabSim Simulation Environment - Separate Videos')
    parser.add_argument('--backend', type=str, default='numpy',
                       choices=['numpy', 'gpu'],
                       help='Backend choice: numpy (CPU) or gpu')
    parser.add_argument('--headless', action='store_true',
                       help='Run in headless mode (default is with GUI)')
    parser.add_argument('--no-video', action='store_true',
                       help='Disable video display and saving')
    parser.add_argument('--config-name', type=str, default='level3_Heat_Liquid',
                       help='Configuration file name (without .yaml extension)')
    parser.add_argument('--config-dir', type=str, default='config',
                       help='Configuration directory path (default: config)')
    return parser.parse_args()

# Get command line arguments
args = parse_args()

# Set up simulation app based on arguments
simulation_config = {"headless": args.headless}
simulation_app = SimulationApp(simulation_config)

import hydra
from omegaconf import OmegaConf
import cv2
import numpy as np

import omni
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.usd
from isaacsim.core.utils import extensions

extensions.enable_extension("omni.physx.bundle")
extensions.enable_extension("omni.usdphysics.ui")

from factories.robot_factory import create_robot
from utils.object_utils import ObjectUtils
from factories.task_factory import create_task
from factories.controller_factory import create_controller

def main():
    hydra.initialize(config_path=args.config_dir, job_name=args.config_name)
    cfg = hydra.compose(config_name=args.config_name)
    os.makedirs(cfg.multi_run.run_dir, exist_ok=True)
    OmegaConf.save(cfg, cfg.multi_run.run_dir + "/config.yaml")

    # Set backend based on command line arguments
    if args.backend == 'gpu':
        world = World(stage_units_in_meters=1, device="cpu")
        physx_interface = omni.physx.get_physx_interface()
        physx_interface.overwrite_gpu_setting(1)
    else:
        world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene", backend="numpy")

    # Override configuration based on command line arguments
    if args.no_video:
        save_video = False
        show_video = False
    else:
        save_video = True
        show_video = True

    robot = create_robot(
        cfg.robot.type,
        position=np.array(cfg.robot.position)
    )

    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(usd_path=os.path.abspath(cfg.usd_path), prim_path="/World")

    ObjectUtils.get_instance(stage)

    task = create_task(
        cfg.task_type,
        cfg=cfg,
        world=world,
        stage=stage,
        robot=robot,
    )

    task_controller = create_controller(
        cfg.controller_type,
        cfg=cfg,
        robot=robot,
    )

    # Multiple video writers, one for each camera
    video_writers = {}  # {camera_name: cv2.VideoWriter}
    task.reset()

    while simulation_app.is_running():
        world.step(render=True)

        if world.is_stopped():
            task_controller.reset_needed = True

        if world.is_playing():
            if task_controller.need_reset() or task.need_reset():
                # Release all video writers
                for writer in video_writers.values():
                    writer.release()
                video_writers.clear()

                task_controller.reset()
                if task_controller.episode_num() >= cfg.max_episodes:
                    task_controller.close()
                    simulation_app.close()
                    cv2.destroyAllWindows()
                    break
                task.reset()

                continue

            state = task.step()
            if state is None:
                continue

            action, done, is_success = task_controller.step(state)
            if action is not None:
                robot.get_articulation_controller().apply_action(action)
            if done:
                task.on_task_complete(is_success)
                continue

            if save_video or show_video:
                # Get camera images from state
                camera_images = []
                camera_names = []

                for camera_name, image_data in state['camera_display'].items():
                    # Convert from RGB to BGR for OpenCV
                    display_img = cv2.cvtColor(image_data.transpose(1, 2, 0), cv2.COLOR_RGB2BGR)
                    camera_images.append(display_img)
                    camera_names.append(camera_name)

                # Create episode directory
                episode_dir = os.path.join(cfg.multi_run.run_dir, "video", f"episode_{task_controller._episode_num}")
                os.makedirs(episode_dir, exist_ok=True)

                # Process each camera separately
                for idx, (camera_name, img) in enumerate(zip(camera_names, camera_images)):
                    # Add camera label to image
                    label = f"Camera {idx+1} ({cfg.cameras[idx].image_type})"
                    img_labeled = img.copy()
                    cv2.putText(img_labeled, label, (2, 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1)

                    # Show video if enabled (stack all cameras horizontally for display only)
                    if show_video:
                        if idx == 0:
                            combined_img = img_labeled
                        else:
                            combined_img = np.hstack([combined_img, img_labeled])
                        if idx == len(camera_images) - 1:
                            cv2.imshow('Camera Views', combined_img)
                            cv2.waitKey(1)

                    # Save each camera to separate video file
                    if save_video:
                        output_path = os.path.join(episode_dir, f"camera_{idx+1}_{cfg.cameras[idx].image_type}.mp4")

                        # Create video writer if not exists for this camera
                        if camera_name not in video_writers:
                            height, width = img_labeled.shape[:2]
                            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                            video_writers[camera_name] = cv2.VideoWriter(output_path, fourcc, 60.0, (width, height))

                        # Write frame to the appropriate video writer
                        video_writers[camera_name].write(img_labeled)


if __name__ == "__main__":
    main()
