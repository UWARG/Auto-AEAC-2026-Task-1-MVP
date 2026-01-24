import depthai as dai
print("dai.XLinkOut exists?", hasattr(dai, 'XLinkOut'))
if hasattr(dai, 'node'):
    print("dai.node.XLinkOut exists?", hasattr(dai.node, 'XLinkOut'))
    
# Search for XLinkOut
import inspect
def find_xlink(obj, path="dai"):
    for name in dir(obj):
        if "XLink" in name:
            print(f"Found {name} in {path}")

find_xlink(dai)
if hasattr(dai, 'node'):
    find_xlink(dai.node, "dai.node")
