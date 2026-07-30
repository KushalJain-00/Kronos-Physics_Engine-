from scenes.pendulum import build as pendulum
from scenes.hinge_joint import build as hinge_joint
from scenes.rope import build as rope
from scenes.stacking_test import build as stacking_test

SCENES = {
    "pendulum": pendulum,
    "hinge_joint": hinge_joint,
    "rope": rope,
    "stacking_test": stacking_test,
}