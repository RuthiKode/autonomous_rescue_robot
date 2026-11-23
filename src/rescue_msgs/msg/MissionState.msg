# Mission state, published by the mission behaviour tree on /mission/state at every change.

string STATE_STARTING="STARTING"
string STATE_EXPLORING="EXPLORING"
string STATE_VERIFYING="VERIFYING"
string STATE_RECOVERING="RECOVERING"
string STATE_DONE="DONE"
string STATE_FAILED="FAILED"

builtin_interfaces/Time stamp
string state          # one of the STATE_* constants
string reason         # why the state changed, in words
uint32 goals_sent     # frontier goals sent to Nav2 so far
uint32 goals_failed   # of those, how many did not reach the goal
uint32 recoveries     # recovery behaviours run so far (stuck, localization jump)
uint32 verifications  # verification visits made so far
