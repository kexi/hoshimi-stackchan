"""CoreS3をリセットさせずにUSBシリアルを開く。"""

import serial


class PassiveControlLineSerial(serial.Serial):
    """監視用接続ではDTR/RTSの状態を変更しない。"""

    def _update_dtr_state(self):
        return

    def _update_rts_state(self):
        return


def open_serial_without_reset(port: str, baudrate: int = 115200, timeout: float = 0.5):
    """DTR/RTSを一切変更せずにポートを開く。"""
    connection = PassiveControlLineSerial()
    connection.port = port
    connection.baudrate = baudrate
    connection.timeout = timeout
    # Why not DTR/RTS=False: pyserialはopen()中に両線を別々のioctlで反映する。
    # 最終値が解除でも途中状態をESP32-S3のUSB Serial/JTAGがリセットとして拾う。
    # 監視と時刻同期には制御線が不要なので、既存状態へ触らない。
    connection.dtr = False
    connection.rts = False
    connection.open()
    return connection
