import argparse

from lecontrol.control import run_robot

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control the So100 Robot.")
    parser.add_argument("--mode", type=str, default="teleoperation", help="Operation mode.")
    parser.add_argument("--record_dataset", type=bool, default=False, help="Record dataset?")
    parser.add_argument("--dataset_task", type=str, default="", help="Dataset task name.")
    parser.add_argument("--replay_episode", type=str, default="", help="Replay a .npy episode.")
    args = parser.parse_args()

    run_robot(
        mode=args.mode,
        record_dataset=args.record_dataset,
        replay_episode=args.replay_episode,
        dataset_task=args.dataset_task
    )
