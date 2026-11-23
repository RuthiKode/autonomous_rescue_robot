from rescue_mission.logic import DONE, FAILED, GO, RETRY, WAIT, ExploreLogic


def test_found_goal_is_sent():
    lg = ExploreLogic()
    assert lg.on_selection(True, 'ok', (3.0, 1.0)) == GO


def test_no_map_waits_without_counting_towards_done():
    lg = ExploreLogic(confirm_empty=2)
    for _ in range(5):
        assert lg.on_selection(False, 'no_map', None) == WAIT
    assert lg.empty_count == 0


def test_done_only_after_consecutive_empty_answers():
    lg = ExploreLogic(confirm_empty=3)
    assert lg.on_selection(False, 'no_frontiers', None) == WAIT
    assert lg.on_selection(False, 'no_frontiers', None) == WAIT
    assert lg.on_selection(True, 'ok', (1.0, 1.0)) == GO          # a frontier reappeared: reset
    assert lg.empty_count == 0
    for _ in range(2):
        assert lg.on_selection(False, 'no_frontiers', None) == WAIT
    assert lg.on_selection(False, 'no_frontiers', None) == DONE
    assert 'no frontier' in lg.done_reason


def test_unreachable_frontiers_take_longer_to_give_up_and_clear_the_costmaps_once():
    lg = ExploreLogic(confirm_empty=1, confirm_unreachable=4)
    assert lg.on_selection(False, "no_reachable_frontier", None) == WAIT
    assert not lg.want_clear
    assert lg.on_selection(False, "no_reachable_frontier", None) == WAIT
    assert lg.want_clear                               # halfway: ask for a costmap clear
    assert lg.on_selection(False, "no_reachable_frontier", None) == WAIT
    assert lg.on_selection(False, "no_reachable_frontier", None) == DONE
    assert "reachable" in lg.done_reason
    lg2 = ExploreLogic(confirm_unreachable=4)
    lg2.on_selection(False, "no_reachable_frontier", None)
    assert lg2.on_selection(True, "ok", (1.0, 1.0)) == GO     # a path appeared: the count restarts
    assert lg2.unreachable_count == 0


def test_failed_goal_is_blacklisted_and_counted():
    lg = ExploreLogic()
    lg.on_goal_sent()
    lg.on_result((2.0, 2.0), False)
    assert lg.blacklist == [(2.0, 2.0)] and lg.goals_failed == 1 and lg.consecutive_failures == 1
    lg.on_goal_sent()
    lg.on_result((4.0, 2.0), True)
    assert lg.consecutive_failures == 0 and lg.reached == [(4.0, 2.0)]


def test_a_goal_chosen_again_after_being_reached_is_blacklisted():
    lg = ExploreLogic(repeat_radius_m=1.0)
    lg.on_goal_sent()
    lg.on_result((4.0, 2.0), True)
    assert lg.on_selection(True, 'ok', (4.3, 2.2)) == RETRY
    assert (4.3, 2.2) in lg.blacklist
    assert lg.on_selection(True, 'ok', (8.0, 2.0)) == GO


def test_repeated_failures_stop_the_mission():
    lg = ExploreLogic(max_consecutive_failures=3)
    for i in range(3):
        lg.on_goal_sent()
        lg.on_result((float(i), 0.0), False)
    assert lg.on_selection(True, 'ok', (9.0, 9.0)) == FAILED
    assert '3 goals failed in a row' in lg.fail_reason


def test_an_interrupted_goal_is_not_blamed():
    lg = ExploreLogic()
    lg.on_goal_sent()
    lg.on_result((2.0, 2.0), False, interrupted=True)
    assert lg.blacklist == [] and lg.goals_failed == 0 and lg.consecutive_failures == 0


def test_repeated_recoveries_without_progress_stop_the_mission():
    lg = ExploreLogic(max_consecutive_recoveries=3)
    for _ in range(3):
        lg.on_recovery()
    assert lg.on_selection(True, 'ok', (1.0, 1.0)) == FAILED
    assert 'recoveries in a row' in lg.fail_reason
    lg2 = ExploreLogic(max_consecutive_recoveries=3)
    lg2.on_recovery()
    lg2.on_recovery()
    lg2.on_result((1.0, 1.0), True)                  # a goal was reached: the count restarts
    assert lg2.consecutive_recoveries == 0 and lg2.recoveries == 2


def test_verify_logic_picks_the_nearest_eligible_pin_and_limits_attempts():
    from rescue_mission.logic import VerifyLogic
    v = VerifyLogic(min_hits=12, max_attempts=2)
    tracks = [(1, 5.0, 0.0, 'confirmed', 40), (2, 1.0, 0.0, 'candidate', 3),
              (3, 2.0, 0.0, 'candidate', 20), (4, 0.5, 0.0, 'merged', 99),
              (5, 0.4, 0.0, 'rejected', 99), (6, 0.3, 0.0, 'verified', 99)]
    assert v.pick(tracks, (0.0, 0.0))[0] == 3         # candidate 2 has too few hits; 4-6 are not eligible
    v.on_visit(3)
    assert v.pick(tracks, (0.0, 0.0))[0] == 3         # one attempt left
    v.on_visit(3)
    assert v.pick(tracks, (0.0, 0.0))[0] == 1
    v.on_visit(1)
    v.on_visit(1)
    assert v.pick(tracks, (0.0, 0.0)) is None and v.visits == 4
