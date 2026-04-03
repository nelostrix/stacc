# ══════════════════════════════════════════
# DEPLOY HELPER
# ══════════════════════════════════════════
from __future__ import annotations

import os
import sys
import time
from typing import Any, Dict, List, Optional


class Deployer:
    """Deploy stacc projects to remote robots via SSH.

    Usage:
        deployer = Deployer("192.168.1.100", user="pi")
        deployer.upload("./build/rpi/")
        deployer.run("python3 src/robot.py")
    """

    def __init__(
        self,
        host: str,
        user: str = "pi",
        port: int = 22,
        key_file: Optional[str] = None,
    ):
        self.host = host
        self.user = user
        self.port = port
        self.key_file = key_file
        self._ssh = None

    def connect(self) -> bool:
        try:
            import paramiko

            self._ssh = paramiko.SSHClient()
            self._ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            kwargs = {"hostname": self.host, "username": self.user, "port": self.port}
            if self.key_file:
                kwargs["key_filename"] = self.key_file
            self._ssh.connect(**kwargs)
            return True
        except Exception as e:
            print(f"  [deploy] SSH connection failed: {e}")
            return False

    def upload(self, local_path: str, remote_path: str = "/home/pi/stacc_robot/"):
        if not self._ssh:
            return
        try:
            import paramiko

            sftp = self._ssh.open_sftp()
            self._upload_recursive(sftp, local_path, remote_path)
            sftp.close()
            print(f"  [deploy] Uploaded {local_path} → {remote_path}")
        except Exception as e:
            print(f"  [deploy] Upload failed: {e}")

    def _upload_recursive(self, sftp, local, remote):
        import os

        if os.path.isfile(local):
            try:
                sftp.stat(os.path.dirname(remote))
            except FileNotFoundError:
                self._mkdir_p(sftp, os.path.dirname(remote))
            sftp.put(local, remote)
        elif os.path.isdir(local):
            try:
                sftp.stat(remote)
            except FileNotFoundError:
                sftp.mkdir(remote)
            for item in os.listdir(local):
                self._upload_recursive(
                    sftp, os.path.join(local, item), os.path.join(remote, item)
                )

    def _mkdir_p(self, sftp, path):
        parts = path.split("/")
        current = ""
        for part in parts:
            if not part:
                current = "/"
                continue
            current = current + "/" + part if current != "/" else "/" + part
            try:
                sftp.stat(current)
            except FileNotFoundError:
                sftp.mkdir(current)

    def run(self, command: str, timeout: float = 30.0) -> str:
        if not self._ssh:
            return ""
        stdin, stdout, stderr = self._ssh.exec_command(command, timeout=timeout)
        output = stdout.read().decode("utf-8")
        errors = stderr.read().decode("utf-8")
        if errors:
            print(f"  [deploy] stderr: {errors}")
        return output

    def install_stacc(self):
        """Install stacc on the remote robot."""
        print("  [deploy] Installing stacc on remote...")
        self.run(
            "pip3 install stacc --break-system-packages 2>/dev/null || pip3 install stacc"
        )

    def close(self):
        if self._ssh:
            self._ssh.close()
