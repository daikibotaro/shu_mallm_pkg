import sys
import os

# 絶対パスでrag/src/ディレクトリをsys.pathに追加
path_to_add = '/home/hiratalab/catkin_ws/src/shu_task_planning/rag/src'
print(f"Adding path to sys.path: {path_to_add}")
sys.path.append(path_to_add)

# knowledge_base.py がそのパスに存在しているか確認
if os.path.exists(os.path.join(path_to_add, 'knowledge_base.py')):
    print("knowledge_base.py found.")
else:
    print("knowledge_base.py NOT found. Check the path!")

# その後にモジュールをインポート
try:
    from knowledge_base import KnowledgeBase
    print("Module imported successfully.")
except ModuleNotFoundError as e:
    print(f"Error importing module: {e}")
