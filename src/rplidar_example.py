from scanner import RPLidar
import asyncio

lidar = RPLidar("/dev/ttyUSB0", 460800)

async def process_scan_data():
    # Start a scan with dictionary output
    async with asyncio.TaskGroup() as tg:
        # Create a task to stop scanning after 5 seconds
        tg.create_task(wait_and_stop(5, lidar.stop_event))
        
        # Create a task to process data from the queue
        tg.create_task(process_queue(lidar.output_queue, lidar.stop_event))
        
        # Start the scan with dictionary output
        tg.create_task(lidar.simple_scan(make_return_dict=True))
    
    # Access the scan data dictionary
    print(lidar.output_dict)
    
    # Reset the device
    lidar.reset()

async def wait_and_stop(seconds, event):
    await asyncio.sleep(seconds)
    event.set()

async def process_queue(queue, stop_event):
    while not stop_event.is_set():
        if not queue.empty():
            data = await queue.get()
            # Process the data
            print(f"Angle: {data['a_deg']}°, Distance: {data['d_mm']}mm, Quality: {data['q']}")
        else:
            await asyncio.sleep(0.1)

# Run the example
try:
    asyncio.run(process_scan_data())
except KeyboardInterrupt:
    lidar.reset()
