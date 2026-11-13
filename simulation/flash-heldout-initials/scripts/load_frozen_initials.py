"""Read frozen evaluation inputs without importing a simulator or a policy."""
import copy
import hashlib
import json
import math
from pathlib import Path


def state_hash(state):
    return hashlib.sha256(json.dumps(state, sort_keys=True, allow_nan=False).encode()).hexdigest()


def load_frozen_initials(path, *, expected_sha256):
    """Return independent copies of flat reference-v2 states in frozen test order.

    Pass each initial_state through the existing reference-v2 tensor conversion
    before env.reset. A fresh native mj_resetData is also required by the plan.
    This reader does not change the simulator, fit normalizers, or warm up physics.
    """
    raw = Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected_sha256:
        raise ValueError('Frozen initial-state file SHA256 mismatch')
    dataset = json.loads(raw)
    if (dataset['schema_version'] != 1 or dataset['count'] != 50
            or dataset['task'] != 'libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate'
            or len(dataset['states']) != 50):
        raise ValueError('Unexpected frozen initial-state schema/task/count')
    hashes = set()
    for index, row in enumerate(dataset['states']):
        state = row['initial_state']
        if row['test_id'] != index or state_hash(state) != row['state_sha256']:
            raise ValueError('State order or state SHA256 mismatch')
        if row['state_sha256'] in hashes:
            raise ValueError('Duplicate state')
        hashes.add(row['state_sha256'])
        if set(state) != {'franka', 'wooden_cabinet', 'akita_black_bowl', 'plate'}:
            raise ValueError('Unexpected scene entities')
        for entity in state.values():
            for key, size in [('pos', 3), ('rot', 4)]:
                if len(entity[key]) != size or not all(math.isfinite(v) for v in entity[key]):
                    raise ValueError('Invalid pose')
    return copy.deepcopy(dataset['states'])
