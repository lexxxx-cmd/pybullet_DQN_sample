import pybullet as p
import pybullet_data
import time

class block:
    def __init__(self, client, base):
        self.shift = [0, -0.02, 0]
        self.scale = [0.3, 0.3, 0.3]

        # 创建视觉形状和碰撞箱形状
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName="duck.obj",
            rgbaColor=[1, 0.5, 1, 1],
            specularColor=[0.4, 0.4, 0],
            visualFramePosition=self.shift,
            meshScale=self.scale)

        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName="duck_vhacd.obj",
            collisionFramePosition=self.shift,
            meshScale=self.scale)

        # 使用创建的视觉形状和碰撞箱形状使用createMultiBody将两者结合在一起
        p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[base[0], base[1], 2],
            useMaximalCoordinates=True)
