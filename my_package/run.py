import subprocess

process1 = subprocess.Popen(["python3", "/home/hiratalab/catkin_ws/src/shu_mallm_pkg/my_package/image_url.py"])
# ローカルホストでポートを開く
# process2 = subprocess.Popen(["chainlit", "run", "main.py", "-w"], cwd="/home/hiratalab/catkin_ws/src/shu_mallm_pkg/my_package/")
# サーバーをネットワーク全体に公開する
process2 = subprocess.Popen(["chainlit", "run", "main.py", "-w", "--host", "0.0.0.0"], cwd="/home/hiratalab/catkin_ws/src/shu_mallm_pkg/my_package/")
# 全てのプロセスが終了するまで待つ
process1.wait()
process2.wait()
