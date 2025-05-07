import argparse

from lecontrol.control import run_robot

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control the So100 Robot.")
    parser.add_argument("--mode", type=str, default="teleoperation", help="Operation mode.")
    parser.add_argument("--dataset_path", type=str, default="", help="Dataset path.")
    parser.add_argument("--replay_episode", type=str, default="", help="Replay a .npy episode.")
    args = parser.parse_args()

    run_robot(
        mode=args.mode,
        replay_episode=args.replay_episode,
        dataset_path=args.dataset_path
    )
