from tools.qc.common import (
    inset_polygon,
    point_in_polygon,
    polygon_self_intersections,
    seg_clears_circles,
    seg_in_polygon,
)


def test_click_order_bowtie_is_detected():
    # Same failure class as the May 29 field smoke: corners in click order can
    # make a self-intersecting polygon even though the physical rectangle is OK.
    bowtie = [(0.0, 0.0), (4.0, 4.0), (0.0, 4.0), (4.0, 0.0)]
    assert polygon_self_intersections(bowtie)


def test_ccw_rectangle_contains_and_insets_points():
    rect = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]
    inset = inset_polygon(rect, 0.5)
    assert point_in_polygon((2.0, 1.5), inset)
    assert not point_in_polygon((0.1, 0.1), inset)


def test_segments_validate_against_area_and_exclusion():
    poly = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]
    assert seg_in_polygon((1.0, 1.0), (4.0, 4.0), poly)
    assert not seg_in_polygon((1.0, 1.0), (6.0, 4.0), poly)
    assert seg_clears_circles((0.5, 0.5), (0.5, 4.5), [(3.0, 3.0, 0.5)])
    assert not seg_clears_circles((0.5, 0.5), (4.5, 4.5), [(2.5, 2.5, 0.5)])


def test_convex_rectangle_has_no_self_intersections():
    # Negative case complementing the bow-tie: a clean CCW rectangle is fine.
    rect = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]
    assert polygon_self_intersections(rect) == []


def test_inset_keeps_vertex_coincident_with_centroid():
    # Exercises the d<1e-9 guard in inset_polygon: a vertex sitting exactly at
    # the centroid has no inward direction, so it must be left in place.
    poly = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (1.0, 1.0)]
    inset = inset_polygon(poly, 0.3)  # centroid is (1.0, 1.0) == last vertex
    assert inset[4] == (1.0, 1.0)


def test_seg_clears_circles_grazing_just_inside_and_outside():
    # A vertical segment at x=2 vs a circle centred at (1, 1): clears at r=0.9,
    # fails once r grows past the 1.0 gap to the segment.
    seg = ((2.0, -1.0), (2.0, 3.0))
    assert seg_clears_circles(seg[0], seg[1], [(1.0, 1.0, 0.9)])
    assert not seg_clears_circles(seg[0], seg[1], [(1.0, 1.0, 1.1)])
