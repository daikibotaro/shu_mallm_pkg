import subprocess


class ChainlitRunner:
    @staticmethod
    def run_chainlit():
        try:
            result = subprocess.run(["python3", "/home/hiratalab/catkin_ws/src/shu_mallm_pkg/my_package/run.py"])
            print("Subprocess output: %s", result.stdout)
        except subprocess.CalledProcessError as e:
            print("Subprocess failed with error: %s", e)
