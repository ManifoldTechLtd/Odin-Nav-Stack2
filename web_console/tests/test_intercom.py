import time

import pytest

from web_console.intercom import AudioIntercom


def wait_idle(intercom, timeout=5.0):
    end = time.time() + timeout
    while intercom.get_status()["pending"] and time.time() < end:
        time.sleep(0.02)


def test_tts_is_queued_and_returns_immediately(tmp_path):
    played = []

    def slow_tts(text, out_wav):
        time.sleep(0.5)
        with open(out_wav, "wb") as f:
            f.write(b"x" * 1024)

    ic = AudioIntercom(cache_dir=str(tmp_path), custom_tts_fn=slow_tts,
                       custom_play_fn=lambda path, meta: played.append(meta))
    started = time.time()
    res = ic.speak_tts_async("你好")
    assert res["queued"] is True
    assert time.time() - started < 0.2
    wait_idle(ic)
    assert played and played[0]["text"] == "你好"
    assert ic.get_status()["last_error"] is None


def test_tts_failure_is_reported_in_status(tmp_path):
    def broken_tts(text, out_wav):
        raise RuntimeError("合成失败")

    ic = AudioIntercom(cache_dir=str(tmp_path), custom_tts_fn=broken_tts, custom_play_fn=lambda p, m: None)
    ic.speak_tts_async("谢谢配合")
    wait_idle(ic)
    assert "合成失败" in ic.get_status()["last_error"]


def test_empty_text_is_rejected_synchronously(tmp_path):
    ic = AudioIntercom(cache_dir=str(tmp_path), custom_play_fn=lambda p, m: None)
    with pytest.raises(ValueError):
        ic.speak_tts_async("   ")


def test_raw_audio_is_queued(tmp_path):
    ic = AudioIntercom(cache_dir=str(tmp_path), custom_play_fn=lambda path, meta: None)
    res = ic.speak_raw_async(b"\x00" * 128, mime_or_ext="webm", volume=100)
    assert res["queued"] is True
    wait_idle(ic)
    assert ic.get_status()["pending"] == 0
