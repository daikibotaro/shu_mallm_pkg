from multi_agent.agents.base_agent import BaseAgent
from autogen import UserProxyAgent
from typing import Dict, Any, List, Optional
import os
import tempfile
import subprocess


class ExecutorAgent(UserProxyAgent):
    """コードの実行と監視を担当するエージェント"""

    def __init__(self, config: Dict[str, Any]):
        name = config.get("name", "ExecutorAgent")
        system_message = config.get("system_message", "生成されたコードの実行と監視を行います。")

        # UserProxyAgentとして初期化
        super().__init__(
            name=name,
            system_message=system_message,
            human_input_mode="NEVER",
            code_execution_config={
                "work_dir": config.get("work_dir", "execution_temp"),
                "use_docker": config.get("use_docker", False)
            }
        )

        self.file_manager = None  # 後で初期化

    def set_file_manager(self, file_manager):
        """FileManagerの参照を設定"""
        self.file_manager = file_manager

    async def execute_code(self, code: str) -> Dict[str, Any]:
        """コードを実行

        Args:
            code: 実行するPythonコード

        Returns:
            実行結果
        """
        # 一時ファイルにコードを保存
        with tempfile.NamedTemporaryFile(suffix='.py', delete=False) as f:
            f.write(code.encode('utf-8'))
            temp_file = f.name

        try:
            # サブプロセスとしてコード実行（実際にはDisplayManagerを通じてユーザー確認後）
            if self.file_manager:
                execution_approved = await self.file_manager.check_create_file(code)
                if not execution_approved:
                    return {
                        "success": False,
                        "message": "ユーザーが実行をキャンセルしました。",
                        "logs": ""
                    }

                # 結果をFileManagerから取得（実際の実装では結果取得の仕組みが必要）
                return {
                    "success": True,
                    "message": "コードが正常に実行されました。",
                    "logs": "実行ログはFileManagerで管理されています。"
                }

            # FileManagerが利用できない場合のフォールバック
            result = subprocess.run(['python', temp_file],
                                    capture_output=True,
                                    text=True)

            return {
                "success": result.returncode == 0,
                "message": "実行完了" if result.returncode == 0 else "実行中にエラーが発生しました",
                "logs": result.stdout,
                "errors": result.stderr
            }

        except Exception as e:
            return {
                "success": False,
                "message": f"実行中に例外が発生しました: {str(e)}",
                "logs": "",
                "errors": str(e)
            }
        finally:
            # 一時ファイルの削除
            if os.path.exists(temp_file):
                os.unlink(temp_file)

    async def monitor_execution(self, execution_id: str) -> Dict[str, Any]:
        """実行の監視

        Args:
            execution_id: 実行ID

        Returns:
            監視結果
        """
        # 実際の実装では実行状態の監視方法が必要
        # ここでは簡略化のためモック実装
        return {
            "status": "completed",
            "execution_time": 5.2,
            "resource_usage": {
                "cpu": "25%",
                "memory": "150MB"
            },
            "completion_status": "success"
        }
