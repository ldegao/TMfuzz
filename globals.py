import os

client = None
tm = None
# this is a small trick of the author before,we may fix it later
town_map = None
username = os.environ['USER']
immobile_percentage = 5  # 5% of the actors are immobile forever
stop_percentage = 5  # 5% of the actors are immobile in stop_seconds
stop_seconds = 5  # seconds
test_split_1 = None
test_split_2 = None

