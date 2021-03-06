class update_test_rosbagmigration_PartiallyMigrated_e6d73341e7b3f15f987c0cf194f97350(MessageUpdateRule):
	old_type = "test_rosbagmigration/PartiallyMigrated"
	old_full_text = """
int32            field1 # 40
MigratedExplicit field2 # (17, 58.2, "aldfkja", 82)

================================================================================
MSG: test_rosbagmigration/MigratedExplicit
Header  header
int32   field1 #17
float32 field2 #58.2
string  field3 #"aldfkja"
int32   field4 #82
================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
"""

	new_type = "test_rosbagmigration/PartiallyMigrated"
	new_full_text = """
int32            field1 # 40
MigratedExplicit field2 # (17, 58.2, "aldfkja", 82)
string           field3 # "radasdk"

================================================================================
MSG: test_rosbagmigration/MigratedExplicit
Header  header
int32   field1 #17
float32 field2 #58.2
string  field3 #"aldfkja"
int32   field4 #82
================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
"""

	order = 0
	migrated_types = [("MigratedExplicit","MigratedExplicit"),]

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.field1 = old_msg.field1
		self.migrate(old_msg.field2, new_msg.field2)
		new_msg.field3 = 'radasdk'
