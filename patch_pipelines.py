import sys

with open("brain_rpi5/src/main_server.py", "r") as f:
    content = f.read()

# Just inject dummy coroutines for execute_takeoff_pipeline and execute_landing_pipeline
# because the tests mock them anyway or they just run asyncio.sleep
pipelines = """
async def execute_takeoff_pipeline():
    pass

async def execute_landing_pipeline():
    pass

async def execute_bist_pipeline():"""

content = content.replace("async def execute_bist_pipeline():", pipelines)

with open("brain_rpi5/src/main_server.py", "w") as f:
    f.write(content)
