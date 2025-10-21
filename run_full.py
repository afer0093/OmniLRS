"""
OmniLRS launcher for Isaac Sim Full 4.5.0 or standard python runtime.
"""

import omni
import runpy
import importlib

def run_in_full():
    """Run OmniLRS inside an already running Isaac Sim Full session."""
    print("[OmniLRS] Detected Isaac Sim Full session. Patching startSim() ...")

    # 1. モジュールの読み込み
    startSimModule = importlib.import_module("src.environments_wrappers.startSim")

    # 2. Full GUI 上でSimulationAppを再生成しないように差し替え
    def startSim_existing(cfg: dict):
        from src.environments.rendering import set_lens_flares, set_chromatic_aberrations, set_motion_blur
        from src.environments_wrappers.ros2.simulation_manager_ros2 import ROS2_SimulationManager
        from src.environments_wrappers.ros1.simulation_manager_ros1 import ROS1_SimulationManager
        from src.environments_wrappers.sdg.simulation_manager_sdg import SDG_SimulationManager

        simulation_app = omni.kit.app.get_app()
        print("[OmniLRS] Using existing SimulationApp from Isaac Sim Full (GUI).")

        # Rendering設定を反映（必要なら）
        set_lens_flares(cfg)
        set_motion_blur(cfg)
        set_chromatic_aberrations(cfg)

        mode = cfg["mode"]["name"]
        if mode == "ROS2":
            SM = ROS2_SimulationManager(cfg, simulation_app)
        elif mode == "ROS1":
            SM = ROS1_SimulationManager(cfg, simulation_app)
        elif mode == "SDG":
            SM = SDG_SimulationManager(cfg, simulation_app)
        else:
            raise ValueError(f"Unknown mode {mode}")
        return SM, simulation_app

    # 3. 差し替え
    startSimModule.startSim = startSim_existing

    # 4. 通常のrun.pyを実行（Hydra含む設定ロードはそのまま）
    runpy.run_path("/workspace/omnilrs/run.py")


def main():
    """Detect whether Isaac Sim Full is running."""
    try:
        app = omni.kit.app.get_app()
        if app is not None:
            run_in_full()
        else:
            raise RuntimeError
    except Exception:
        print("[OmniLRS] No Isaac Sim Full session detected. Running standalone.")
        runpy.run_path("/workspace/omnilrs/run.py")


if __name__ == "__main__":
    main()
