class update_test_rosbagmigration_SimpleMigrated_f3d103d10e4d7f4e5c4b19aa46d9a9dd(MessageUpdateRule):
	old_type = "test_rosbagmigration/SimpleMigrated"
	old_full_text = """
int32 data1 # 42
"""

	new_type = "test_rosbagmigration/SimpleMigrated"
	new_full_text = """
int32 data2 # 42
"""

	order = 0
	migrated_types = [

	]

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.data2 = old_msg.data1

class update_test_rosbagmigration_SimpleMigrated_01dfc3630b3a6d483b2f36047889c82c(MessageUpdateRule):
	old_type = "test_rosbagmigration/SimpleMigrated"
	old_full_text = """
int32 data2 # 42
"""

	new_type = "test_rosbagmigration/SimpleMigrated"
	new_full_text = """
int32 data3 # 42
"""

	order = 1
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.data3 = old_msg.data2

class update_test_rosbagmigration_SimpleMigrated_e6246c5a2c249e1dd0fa12bf54357ad2(MessageUpdateRule):
	old_type = "test_rosbagmigration/SimpleMigrated"
	old_full_text = """
int32 data3 # 42
"""

	new_type = "test_rosbagmigration/SimpleMigrated"
	new_full_text = """
int32 data # 42
"""

	order = 2
	migrated_types = []

	valid = True

	def update(self, old_msg, new_msg):
		new_msg.data = old_msg.data3


