python 01_generate_data.py single_racecar_bullet --total_episodes 2000 --time_steps 300 --render 1 --verbose 1 --verbose_stdout 1 --verbose_stderr 1

python model.py single_racecar_bullet --filename ./controller/single_racecar_bullet.cma.4.12.best.json --render_mode --record_video
