from tdmclient import ClientAsync
import asyncio

async def main():
    with ClientAsync() as client:
        with await client.lock() as node:
            await allign(node, client)

async def allign(node, client):
    await node.wait_for_variables({"motor.left.target", "motor.right.target"})
    # await node.set_variables({
    #     "motor.left.target": [-500],
    #     "motor.right.target": [500]
    # })
    # #await asyncio.sleep(1.0)
    # #free node lock here
    # await client.sleep(5.0)

    # await node.set_variables({
    #     "motor.left.target": [0],
    #     "motor.right.target": [0]
    # })
    # await client.sleep(5.0)
    # await node.set_variables({
    #     "motor.left.target": [100],
    #     "motor.right.target": [100]
    # })
    # #await asyncio.sleep(1.0)
    # #free node lock here
    # await client.sleep(5.0)

    await node.set_variables({
        "motor.left.target": [0],
        "motor.right.target": [0]
    })

    node.unlock()


if __name__ == "__main__":
    asyncio.run(main())