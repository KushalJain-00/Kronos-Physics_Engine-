from scenes.pendulum import build as pendulum
from scenes.hinge_joint import build as hinge_joint
from scenes.rope import build as rope
from scenes.stacking_test import build as stacking_test
from scenes.sandbox import build as sandbox
from scenes.cloth import build as cloth

SCENES = {
    "pendulum": pendulum,
    "hinge_joint": hinge_joint,
    "rope": rope,
    "stacking_test": stacking_test,
    "sandbox": sandbox,
    "cloth": cloth,
}
