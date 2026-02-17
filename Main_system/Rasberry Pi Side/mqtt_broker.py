import asyncio
import logging
from amqtt.broker import Broker

config = {
    'listeners': {
        'default': {
            'type': 'tcp',
            'bind': '0.0.0.0:1883',
        }
    },
    'sys_interval': 10,
    'auth': {
        'allow_anonymous': True,
        'password_file': '',
        'plugins': [
            'auth_anonymous'
        ]
    }
}

async def start_broker():
    broker = Broker(config)
    await broker.start()
    print("MQTT Broker started on port 1883")
    # Keep it running
    while True:
        await asyncio.sleep(1)

if __name__ == '__main__':
    formatter = "[%(asctime)s] :: %(levelname)s :: %(name)s :: %(message)s"
    logging.basicConfig(level=logging.INFO, format=formatter)
    asyncio.run(start_broker())
