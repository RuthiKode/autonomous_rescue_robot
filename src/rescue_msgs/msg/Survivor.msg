# One survivor, either a ground-truth entry or a localizer track.

string STATE_CANDIDATE="candidate"
string STATE_CONFIRMED="confirmed"
string STATE_VERIFIED="verified"

uint32 id                          # unique within one run
geometry_msgs/Pose pose            # in the frame of the enclosing SurvivorArray header (map)
float32 confidence                 # 0..1
uint32 hit_count                   # detections merged into this track
builtin_interfaces/Time first_seen # sim time of the first detection
string state                       # one of the STATE_* constants
