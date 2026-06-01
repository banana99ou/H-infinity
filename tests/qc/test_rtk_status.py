from tools.qc.common import parse_rtk_status


def test_fixed_status_is_authoritative_quality_4():
    s = (
        "FIX: RTK FIXED (quality=4, sats=22, HDOP=0.6, rate=10.0Hz) | "
        "Lat=37.61247000, Lon=126.99426183 | RTCM: OK "
        "(bytes=12345, fwd_age=0.5s, net_age=0.5s)"
    )
    r = parse_rtk_status(s)
    assert r.quality == 4
    assert r.fixed is True
    assert r.float is False
    assert r.rtcm_stale is False
    assert r.fwd_age_s == 0.5


def test_float_and_stale_are_not_fixed():
    r = parse_rtk_status(
        "FIX: RTK FLOAT (quality=5, sats=18) | RTCM: STALE "
        "(bytes=100, fwd_age=42.0s)"
    )
    assert r.quality == 5
    assert r.fixed is False
    assert r.float is True
    assert r.rtcm_stale is True


def test_malformed_status_is_safe_not_fixed():
    r = parse_rtk_status("no useful tokens")
    assert r.quality is None
    assert r.fixed is False
    assert r.float is False


def test_empty_string_is_safe():
    r = parse_rtk_status("")
    assert r.quality is None
    assert r.fixed is False
    assert r.rtcm_stale is False
    assert r.fwd_age_s is None


def test_quality_token_without_number_yields_none():
    r = parse_rtk_status("FIX: RTK (quality=, sats=10)")
    assert r.quality is None
    assert r.fixed is False


def test_age_threshold_is_strict_five_seconds():
    # stale iff age > 5.0 (the RTCM: STALE text aside). 5.0 is NOT stale.
    assert parse_rtk_status("quality=4 (fwd_age=5.0s)").rtcm_stale is False
    assert parse_rtk_status("quality=4 (fwd_age=5.1s)").rtcm_stale is True


def test_stale_text_marks_stale_even_with_fresh_age():
    r = parse_rtk_status("quality=4 RTCM: STALE (fwd_age=0.2s)")
    assert r.rtcm_stale is True
    assert r.fixed is True  # quality gate is independent of RTCM staleness
