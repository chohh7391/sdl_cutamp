import os

from cutamp.envs import TAMPEnvironment
from cutamp.envs.book_shelf import load_book_shelf_env
from cutamp.envs.tetris import load_tetris_env
from cutamp.envs.utils import get_env_dir, load_env
from cutamp.envs.stirring import load_stirring_env    
from cutamp.envs.pouring import load_pouring_env

def load_demo_env(name: str) -> TAMPEnvironment:
    if name.startswith("tetris_"):
        num_blocks = int(name.split("tetris_")[-1])
        env = load_tetris_env(num_blocks, buffer_multiplier=1.0)
    elif name == "book_shelf":
        env = load_book_shelf_env()
    elif name == "blocks":
        env_path = os.path.join(get_env_dir(), "obstacle_blocks_large_region.yml")
        env = load_env(env_path)
    elif name == "unpack":
        env_path = os.path.join(get_env_dir(), "unpack_3.yml")
        env = load_env(env_path)
    elif name == "pouring":
        env = load_pouring_env()
    elif name == "stirring":
        env = load_stirring_env()
    else:
        raise ValueError(f"Unknown environment name: {name}")
    return env