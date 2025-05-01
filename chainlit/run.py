import subprocess

process1 = subprocess.Popen(
    ["python3", "./image_url.py"])
process2 = subprocess.Popen(
    ["chainlit", "run", "demo.py", "-w"])

# 全てのプロセスが終了するまで待つ
process1.wait()
process2.wait()
