import argparse
import json
import time
from pathlib import Path

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback, CallbackList
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.utils import set_random_seed
except ImportError as exc:
    raise ImportError(
        'train_rl requires stable-baselines3. Install it in the runtime environment before launching training.'
    ) from exc

from rl_agent.ros_rl_env import RosTargetTrackingEnv


class PeriodicSaveCallback(BaseCallback):
    def __init__(self, exp_dir: Path, save_freq: int) -> None:
        super().__init__()
        self.exp_dir = exp_dir
        self.save_freq = max(int(save_freq), 1)
        self.ckpt_dir = exp_dir / 'checkpoints'
        self.ckpt_dir.mkdir(parents=True, exist_ok=True)

    def _on_step(self) -> bool:
        if self.num_timesteps % self.save_freq != 0:
            return True
        latest_path = self.exp_dir / 'latest_model.zip'
        ckpt_path = self.ckpt_dir / f'step_{self.num_timesteps}.zip'
        self.model.save(str(latest_path))
        self.model.save(str(ckpt_path))
        return True

    def _on_training_end(self) -> None:
        self.model.save(str(self.exp_dir / 'latest_model.zip'))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Train a PPO target-tracking policy over ROS topics.')
    parser.add_argument('--exp-dir', default='runs/ppo_target_tracking', help='experiment output directory')
    parser.add_argument('--resume-from', default='', help='existing model zip path to resume from')
    parser.add_argument('--total-timesteps', type=int, default=200000, help='additional timesteps to train')
    parser.add_argument('--save-freq', type=int, default=10000, help='checkpoint interval in timesteps')
    parser.add_argument('--seed', type=int, default=42, help='random seed')
    parser.add_argument('--device', default='cpu', help='SB3 device string')
    parser.add_argument(
        '--progress-bar',
        action='store_true',
        help='enable SB3 progress bar (requires tqdm and rich)',
    )
    parser.add_argument('--learning-rate', type=float, default=3e-4)
    parser.add_argument('--n-steps', type=int, default=1024)
    parser.add_argument('--batch-size', type=int, default=256)
    parser.add_argument('--n-epochs', type=int, default=10)
    parser.add_argument('--gamma', type=float, default=0.99)
    parser.add_argument('--gae-lambda', type=float, default=0.95)
    parser.add_argument('--clip-range', type=float, default=0.2)
    parser.add_argument('--ent-coef', type=float, default=0.0)
    parser.add_argument('--vf-coef', type=float, default=0.5)
    parser.add_argument('--max-grad-norm', type=float, default=0.5)
    parser.add_argument('--obs-dim', type=int, default=14)
    parser.add_argument('--step-timeout-sec', type=float, default=1.0)
    parser.add_argument('--reset-timeout-sec', type=float, default=90.0)
    parser.add_argument('--reset-retry-sleep-sec', type=float, default=1.0)
    parser.add_argument('--post-step-settle-sec', type=float, default=0.03)
    parser.add_argument('--max-vx', type=float, default=1.0)
    parser.add_argument('--max-vy', type=float, default=0.4)
    parser.add_argument('--max-vz', type=float, default=1.0)
    parser.add_argument('--max-yaw-rate', type=float, default=1.6)
    parser.add_argument('--tensorboard-log', default='', help='tensorboard log root; empty disables it')
    return parser


def save_run_metadata(args: argparse.Namespace, exp_dir: Path) -> None:
    exp_dir.mkdir(parents=True, exist_ok=True)
    metadata = vars(args).copy()
    metadata['created_at_epoch_sec'] = time.time()
    with (exp_dir / 'config.json').open('w', encoding='utf-8') as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    exp_dir = Path(args.exp_dir).expanduser().resolve()
    exp_dir.mkdir(parents=True, exist_ok=True)
    save_run_metadata(args, exp_dir)
    set_random_seed(args.seed)

    env = RosTargetTrackingEnv(
        obs_dim=args.obs_dim,
        step_timeout_sec=args.step_timeout_sec,
        reset_timeout_sec=args.reset_timeout_sec,
        reset_retry_sleep_sec=args.reset_retry_sleep_sec,
        post_step_settle_sec=args.post_step_settle_sec,
        max_vx=args.max_vx,
        max_vy=args.max_vy,
        max_vz=args.max_vz,
        max_yaw_rate=args.max_yaw_rate,
    )
    env = Monitor(env)

    tensorboard_log = args.tensorboard_log or None
    model_kwargs = {
        'policy': 'MlpPolicy',
        'env': env,
        'learning_rate': args.learning_rate,
        'n_steps': args.n_steps,
        'batch_size': args.batch_size,
        'n_epochs': args.n_epochs,
        'gamma': args.gamma,
        'gae_lambda': args.gae_lambda,
        'clip_range': args.clip_range,
        'ent_coef': args.ent_coef,
        'vf_coef': args.vf_coef,
        'max_grad_norm': args.max_grad_norm,
        'verbose': 1,
        'seed': args.seed,
        'device': args.device,
        'tensorboard_log': tensorboard_log,
    }

    if args.resume_from:
        model = PPO.load(args.resume_from, env=env, device=args.device)
    else:
        model = PPO(**model_kwargs)

    callbacks = CallbackList([
        PeriodicSaveCallback(exp_dir=exp_dir, save_freq=args.save_freq),
    ])

    try:
        model.learn(
            total_timesteps=args.total_timesteps,
            callback=callbacks,
            reset_num_timesteps=not bool(args.resume_from),
            progress_bar=args.progress_bar,
        )
    finally:
        model.save(str(exp_dir / 'latest_model.zip'))
        env.close()


if __name__ == '__main__':
    main()
