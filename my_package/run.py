import subprocess

py310_python = "/home/hiratalab/catkin_ws/src/shu_mallm_pkg/autogen-env/bin/python3.10"

process1 = subprocess.Popen([py310_python, "/home/hiratalab/catkin_ws/src/shu_mallm_pkg/my_package/image_url.py"])
process2 = subprocess.Popen(
    ["chainlit", "run", "main.py", "-w", "--host", "0.0.0.0"],
    cwd="/home/hiratalab/catkin_ws/src/shu_mallm_pkg/my_package/",
    executable=f"/home/hiratalab/catkin_ws/src/shu_mallm_pkg/autogen-env/bin/chainlit"
)

# 全てのプロセスが終了するまで待つ
process1.wait()
process2.wait()
