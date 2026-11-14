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

    Wraps paramiko for SSH/SFTP file transfer and remote command
    execution. Supports context manager usage.

    Usage:
        deployer = Deployer("192.168.1.100", user="pi")
        deployer.upload("./build/rpi/")
        deployer.run("python3 src/robot.py")

    Or as a context manager:
        with Deployer("192.168.1.100") as d:
            d.upload("./build/rpi/")
            d.run("python3 src/robot.py")
    """

    def __init__(
        self,
        host: str,
        user: str = "pi",
        port: int = 22,
        key_file: Optional[str] = None,
        known_hosts: Optional[str] = None,
    ):
        if not host:
            raise ValueError("host must not be empty")

        self.host = host
        self.user = user
        self.port = port
        self.key_file = key_file
        self.known_hosts = known_hosts
        self._ssh = None

    def connect(self) -> bool:
        """Establish SSH connection to the remote host.

        Returns:
            True if connection succeeded
        """
        try:
            import paramiko

            self.close()
            self._ssh = paramiko.SSHClient()
            self._ssh.load_system_host_keys()
            if self.known_hosts:
                self._ssh.load_host_keys(self.known_hosts)
            self._ssh.set_missing_host_key_policy(paramiko.RejectPolicy())
            kwargs: Dict[str, Any] = {
                "hostname": self.host,
                "username": self.user,
                "port": self.port,
                "timeout": 10,
                "banner_timeout": 10,
                "auth_timeout": 10,
            }
            if self.key_file:
                kwargs["key_filename"] = self.key_file
            self._ssh.connect(**kwargs)
            return True
        except Exception as e:
            self.close()
            print(f"  [deploy] SSH connection failed: {e}")
            return False

    def upload(
        self,
        local_path: str,
        remote_path: str = "/home/pi/stacc_robot/",
    ) -> None:
        """Upload a file or directory to the remote host via SFTP.

        Args:
            local_path: Local file or directory path
            remote_path: Destination path on the remote host
        """
        if not self._ssh:
            print("  [deploy] Not connected. Call connect() first.")
            return
        try:
            import paramiko

            sftp = self._ssh.open_sftp()
            self._upload_recursive(sftp, local_path, remote_path)
            sftp.close()
            print(f"  [deploy] Uploaded {local_path} \u2192 {remote_path}")
        except Exception as e:
            print(f"  [deploy] Upload failed: {e}")

    def _upload_recursive(
        self, sftp: Any, local: str, remote: str
    ) -> None:
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
                    sftp,
                    os.path.join(local, item),
                    os.path.join(remote, item),
                )

    @staticmethod
    def _mkdir_p(sftp: Any, path: str) -> None:
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
        """Run a shell command on the remote host.

        Args:
            command: Shell command to execute
            timeout: Command timeout in seconds

        Returns:
            Command stdout as string
        """
        if not self._ssh:
            return ""
        stdin, stdout, stderr = self._ssh.exec_command(command, timeout=timeout)
        output = stdout.read().decode("utf-8")
        errors = stderr.read().decode("utf-8")
        if errors:
            print(f"  [deploy] stderr: {errors}")
        return output

    def install_stacc(self) -> None:
        """Install or update stacc on the remote robot."""
        print("  [deploy] Installing stacc on remote...")
        self.run(
            "pip3 install stacc --break-system-packages 2>/dev/null || pip3 install stacc"
        )

    def close(self) -> None:
        """Close the SSH connection."""
        if self._ssh:
            self._ssh.close()
            self._ssh = None

    def __enter__(self) -> Deployer:
        if not self.connect():
            raise ConnectionError(f"SSH connection to {self.host!r} failed")
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"Deployer(host={self.host!r}, user={self.user!r}, port={self.port})"
